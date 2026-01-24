
#pragma once

#include <string>
#include <frc2/command/SubsystemBase.h>
#include <numbers>

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

// REV SparkMax (new-style) APIs
#include<rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>

// CTRE Phoenix 6 CANcoder
#include <ctre/phoenix6/CANcoder.hpp>

class SwerveModule : public frc2::SubsystemBase {
 public:
  // Primary constructor for CAN SparkMax + CTRE CANcoder
  SwerveModule(int driveMotorCanId,
               int turningMotorCanId,
               int turningCANcoderId,
               units::radian_t turningEncoderOffset,
               units::meters_per_second_t maxModuleSpeed,
               const std::string& name);

  frc::SwerveModuleState GetState() const;
  frc::SwerveModulePosition GetPosition() const;
  void SetDesiredState(frc::SwerveModuleState& state);

  void Periodic() override;

 private:
  // ---- Hardware ----
  rev::spark::SparkMax m_driveMotor;
  rev::spark::SparkMax m_turningMotor;

  // Drive uses SparkMax built-in relative encoder
  rev::spark::SparkRelativeEncoder m_driveEncoder;

  // Turning uses CTRE Phoenix 6 CANcoder (absolute over CAN)
  mutable ctre::phoenix6::hardware::CANcoder m_cancoder;

  // Telemetry keys (optional)
  std::string m_driveSpeedName;
  std::string m_driveAngleName;
  std::string m_driveRawAngleName;

  // ---- SDS MK4 L1 (4" wheel, 8.14:1) ----
  static constexpr double kMetersPerInch = 0.0254;
  static constexpr double kWheelDiameterIn = 4.0;
  static constexpr double kDriveGearRatio = 8.14;
  static constexpr double kWheelCircumferenceMeters =
      kWheelDiameterIn * kMetersPerInch * std::numbers::pi;
  static constexpr double kMetersPerMotorRev =
      kWheelCircumferenceMeters / kDriveGearRatio;
  static constexpr double kRPMtoMps = (1.0 / 60.0) * kMetersPerMotorRev;

  // Reference max speed (used for upstream scaling, if needed)
  units::meters_per_second_t m_maxSpeed{0_mps};

  // ---- Turning controller (Profiled PID with continuous input) ----
  static constexpr auto kModuleMaxAngularVelocity =
      std::numbers::pi * 100_rad_per_s;
  static constexpr auto kModuleMaxAngularAcceleration =
      std::numbers::pi * 200_rad_per_s / 1_s;

  frc::ProfiledPIDController<units::radians> m_turningPIDController{
      0.5, 0.0, 0.0, {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};

  // ---- Drive closed-loop on RIO (PID + feedforward to volts) ----
  frc::PIDController m_drivePID{0.2, 0.0, 0.0};
  frc::SimpleMotorFeedforward<units::meters> m_driveFF{
      0_V,                 // kS (tune if you need to overcome stiction)
      2.0_V / 1_mps        // kV  (start ~1.5–3.0 V per m/s)
      // Optionally add kA later (needs acceleration units include & literals)
  };

  // Helpers
  frc::Rotation2d CurrentModuleRotation() const;
  units::radian_t ReadCancoderAngle() const;

  int m_cancoderId{0};
};
