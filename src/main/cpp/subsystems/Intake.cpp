
// WPILib BSD license in project root.

#include "subsystems/Intake.h"
#include <algorithm>
#include <cmath>

Intake::Intake() {
  ConfigureMotors_();

  // Safe startup
  StopWheels();
  m_angle.Set(0.0);
}

void Intake::ConfigureMotors_() {
  // ---------------- Wheels (velocity control) ----------------
  rev::spark::SparkMaxConfig wcfg;
  wcfg.Inverted(constants::Intake::kWheelsInverted);
  wcfg.SmartCurrentLimit(constants::Intake::kWheelsCurrentLimitA);
  wcfg.VoltageCompensation(constants::Intake::kVoltageCompensation);
  wcfg.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);

  // Velocity loop gains (RPM domain) + FF
  wcfg.closedLoop.P(constants::Intake::kWheelsP)
                 .I(constants::Intake::kWheelsI)
                 .D(constants::Intake::kWheelsD);
  wcfg.closedLoop.feedForward.kS(constants::Intake::kWheelsFF_kS)
                               .kV(constants::Intake::kWheelsFF_kV);

  // Optional: convert velocity to mechanism RPM instead of motor RPM
  wcfg.encoder.VelocityConversionFactor(constants::Intake::kMechRPMperMotorRPM);

  // ---------------- Angle (position control) ----------------
  rev::spark::SparkMaxConfig acfg;
  acfg.Inverted(constants::Intake::kAngleInverted);
  acfg.SmartCurrentLimit(constants::Intake::kAngleCurrentLimitA);
  acfg.VoltageCompensation(constants::Intake::kVoltageCompensation);
  acfg.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);

  // Optional: convert encoder position to mechanism rotations
  acfg.encoder.PositionConversionFactor(constants::Intake::kDegPerMotorRot);

  // Position loop gains + FF (usually FF=0 unless you add gravity/motion FF)
  acfg.closedLoop.P(constants::Intake::kAngleP)
                 .I(constants::Intake::kAngleI)
                 .D(constants::Intake::kAngleD);
  acfg.closedLoop.feedForward.kS(constants::Intake::kAngleFF_kS)
                               .kV(constants::Intake::kAngleFF_kV);

  // Apply configurations
  m_wheels.Configure(wcfg,
                     rev::ResetMode::kResetSafeParameters,
                     rev::PersistMode::kPersistParameters);
  m_wheels.ClearFaults();

  m_angle.Configure(acfg,
                    rev::ResetMode::kResetSafeParameters,
                    rev::PersistMode::kPersistParameters);
  m_angle.ClearFaults();
}

void Intake::SetWheelsSpeedRPM(double rpm) {
  // If you configured encoder conversion, this value becomes mechanism RPM.
  m_wheelsPID.SetSetpoint(rpm, rev::spark::SparkBase::ControlType::kVelocity);
}

void Intake::StopWheels() {
  m_wheelsPID.SetSetpoint(0.0, rev::spark::SparkBase::ControlType::kVelocity);
}

void Intake::SetAngleDeg(double deg) {
  if (!m_homed) return;
  double tgtRot = DegToRot(deg);
  m_angleSetpointRot = tgtRot;
  ApplyAngleSetpoint_();
}

double Intake::GetAngleDeg() const {
  return RotToDeg(m_angleEnc.GetPosition());
}

void Intake::ForceZeroAngleHere() {
  m_angleEnc.SetPosition(0.0);
}

void Intake::StartHoming() {
  m_homed = false;
  m_homingActive = true;
  m_homeStart = frc::Timer::GetFPGATimestamp();
  m_angleSetpointRot.reset();
}

void Intake::RunHoming_() {
  if (!m_homingActive) return;

  // Drive toward the physical stop with switch at the reverse end
  m_angle.Set(constants::Intake::kHomeSpeed);

  // Success: switch pressed
  if (HomeSwitchPressed_()) {
    m_angle.Set(0.0);

    // Define switch position as zero reference
    m_angleEnc.SetPosition(0.0);

    m_homed = true;
    m_homingActive = false;
    return;
  }

  // Timeout protection
  if (frc::Timer::GetFPGATimestamp() - m_homeStart > constants::Intake::kHomeTimeout) {
    m_angle.Set(0.0);
    m_homingActive = false;
    m_homed = false; // not homed
  }
}

void Intake::ApplyAngleSetpoint_() {
  if (!m_homed || !m_angleSetpointRot.has_value()) return;
  m_anglePID.SetSetpoint(*m_angleSetpointRot,
                          rev::spark::SparkBase::ControlType::kPosition);
}

void Intake::PublishTelemetry_() {
  if (!m_publishTelemetry) return;

  IntakeNetTable->PutBoolean("Homed", m_homed);
  IntakeNetTable->PutBoolean("HomingActive", m_homingActive);
  IntakeNetTable->PutBoolean("HomeSwitch", HomeSwitchPressed_());
  IntakeNetTable->PutNumber("AngleDeg", GetAngleDeg());
  IntakeNetTable->PutNumber("AngleRot", m_angleEnc.GetPosition());
  IntakeNetTable->PutNumber("WheelsRPM", m_wheelsEnc.GetVelocity());
}

void Intake::Periodic() {
  if (m_homingActive) {
    RunHoming_();
  } else {
    // Reassert setpoint periodically (helps after brownouts)
    ApplyAngleSetpoint_();
  }
  PublishTelemetry_();
}
