
#include "subsystems/SwerveModule.h"

#include <algorithm>
#include <numbers>

#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/config/SparkMaxConfig.h>

using namespace rev::spark;
using namespace ctre::phoenix6;

SwerveModule::SwerveModule(const int driveMotorCanId,
                           const int turningMotorCanId,
                           const int turningCANcoderId,
                           const units::radian_t turningEncoderOffset,
                           const units::meters_per_second_t maxModuleSpeed,
                           const std::string& name)
    : m_driveMotor(driveMotorCanId, SparkLowLevel::MotorType::kBrushless),
      m_turningMotor(turningMotorCanId, SparkLowLevel::MotorType::kBrushless),
      m_driveEncoder(m_driveMotor.GetEncoder()),
      m_cancoder(turningCANcoderId),
      m_driveSpeedName("DriveSpeed_" + name),
      m_driveAngleName("Angle_" + name),
      m_driveRawAngleName("RawAngle_" + name),
      m_maxSpeed(maxModuleSpeed),
      m_cancoderId(turningCANcoderId) {
  // ---- SparkMax (drive) config ----
  SparkMaxConfig driveCfg{};
  driveCfg.Inverted(false)
          .SetIdleMode(SparkMaxConfig::IdleMode::kBrake)
          .VoltageCompensation(12.0)
          .SmartCurrentLimit(20);

  // Position in meters; velocity in m/s (SDS MK4 L1 factors)
  driveCfg.encoder
      .PositionConversionFactor(kMetersPerMotorRev)
      .VelocityConversionFactor(kRPMtoMps);

  m_driveMotor.Configure(driveCfg,
                         rev::ResetMode::kResetSafeParameters,
                         rev::PersistMode::kPersistParameters);
  m_driveMotor.ClearFaults();
  m_driveEncoder.SetPosition(0.0);

  // ---- SparkMax (turn) config ----
  SparkMaxConfig turnCfg{};
  turnCfg.Inverted(false)
         .SetIdleMode(SparkMaxConfig::IdleMode::kCoast)
         .VoltageCompensation(12.0)
         .SmartCurrentLimit(30, 60);

  m_turningMotor.Configure(turnCfg,
                           rev::ResetMode::kResetSafeParameters,
                           rev::PersistMode::kPersistParameters);
  m_turningMotor.ClearFaults();

  // ---- CANcoder configuration (Phoenix 6) ----
  // Apply magnet offset in *turns*; normalize to [0, 2π) when reading.
  configs::CANcoderConfiguration ccCfg{};
  const double offsetTurns = turningEncoderOffset.value() / (2.0 * std::numbers::pi);
  ccCfg.MagnetSensor.WithMagnetOffset(units::angle::turn_t{offsetTurns});
  // If the angle direction is reversed on your module:
  // ccCfg.MagnetSensor.WithSensorDirection(
  //     signals::SensorDirectionValue::Clockwise_Positive);
  m_cancoder.GetConfigurator().Apply(ccCfg);
  // (CANcoder absolute position returns "turns"; read via StatusSignal API.) [1](https://www.revrobotics.com/rev-21-3005/)

  // Turn PID wraps on [0, 2π)
  m_turningPIDController.EnableContinuousInput(
      0_rad, units::radian_t{2.0 * std::numbers::pi});
}

frc::SwerveModuleState SwerveModule::GetState() const {
  const auto wheelSpeed = units::meters_per_second_t{m_driveEncoder.GetVelocity()};
  const auto angle = ReadCancoderAngle();
  return {wheelSpeed, angle};
}

frc::SwerveModulePosition SwerveModule::GetPosition() const {
  const auto wheelDist = units::meter_t{m_driveEncoder.GetPosition()};
  const auto angle = ReadCancoderAngle();
  return {wheelDist, angle};
}

void SwerveModule::SetDesiredState(frc::SwerveModuleState& referenceState) {
  const frc::Rotation2d encoderRotation = CurrentModuleRotation();

  // WPILib style: minimize rotation & reduce lateral skidding
  referenceState.Optimize(encoderRotation);
  referenceState.CosineScale(encoderRotation);

  // ---- Turning control (Profiled PID) ----
  const auto currentAngle = ReadCancoderAngle();
  const auto turnOutput = m_turningPIDController.Calculate(
      currentAngle, referenceState.angle.Radians());

  // ---- Drive closed-loop (RIO PID + Feedforward -> volts) ----
  const auto currentMps = units::meters_per_second_t{m_driveEncoder.GetVelocity()};
  const auto setpointMps = referenceState.speed;

  const double pidVolts = m_drivePID.Calculate(currentMps.value(), setpointMps.value());
  const auto ffVolts = m_driveFF.Calculate(setpointMps);

  auto totalVolts = units::volt_t{pidVolts} + ffVolts;
  if (totalVolts > 12_V)  totalVolts = 12_V;
  if (totalVolts < -12_V) totalVolts = -12_V;

  m_driveMotor.SetVoltage(units::voltage::volt_t {0.0});
  m_turningMotor.Set(0.0);
}

void SwerveModule::Periodic() {
  // No periodic work required.
}

frc::Rotation2d SwerveModule::CurrentModuleRotation() const {
  return frc::Rotation2d{ReadCancoderAngle()};
}

units::radian_t SwerveModule::ReadCancoderAngle() const {
  // AbsolutePosition returns "turns" (Phoenix 6 StatusSignal); convert to radians. [1](https://www.revrobotics.com/rev-21-3005/)
  const double turns = m_cancoder.GetAbsolutePosition().GetValueAsDouble();
  const double rad   = turns * 2.0 * std::numbers::pi;

  // Normalize to [0, 2π)
  const double twoPi = 2.0 * std::numbers::pi;
  double w = std::fmod(rad, twoPi);
  if (w < 0.0) w += twoPi;
  return units::radian_t{w};
}
