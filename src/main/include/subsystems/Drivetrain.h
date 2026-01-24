
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <numbers>

#include <frc/AnalogGyro.h>  // You can swap to Pigeon2 or navX later if desired
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <networktables/NetworkTable.h>           // added
#include <networktables/NetworkTableInstance.h>

// Units headers (avoid using-directives in headers)
#include <units/angle.h>
#include <units/velocity.h>

#include "SwerveModule.h"

// -----------------------------
// CAN IDs and module constants
// -----------------------------
namespace DriveIds {
// REV SparkMax CAN IDs (drive / turn)
constexpr int kFL_Drive = 1;
constexpr int kFL_Turn  = 2;
constexpr int kFR_Drive = 3;
constexpr int kFR_Turn  = 4;
constexpr int kBL_Drive = 5;
constexpr int kBL_Turn  = 6;
constexpr int kBR_Drive = 7;
constexpr int kBR_Turn  = 8;

// CTRE CANcoder device IDs (fill in your real IDs)
constexpr int kFL_CANCoder = 21;
constexpr int kFR_CANCoder = 22;
constexpr int kBL_CANCoder = 23;
constexpr int kBR_CANCoder = 24;
}  // namespace DriveIds

namespace DriveConst {
// Max wheel speed (pass to each SwerveModule for RIO-side closed-loop scaling)
constexpr auto kMaxModuleSpeed = 4.5_mps;  // must match Drivetrain::kMaxSpeed

// Per-module azimuth zero offsets in radians (measured with wheels pointing robot-forward).
// If you stored MagnetOffset in each CANcoder via Tuner X, leave these at 0_rad.
constexpr auto kFL_Offset = 0.000_rad;
constexpr auto kFR_Offset = 0.000_rad;
constexpr auto kBL_Offset = 0.000_rad;
constexpr auto kBR_Offset = 0.000_rad;
}  // namespace DriveConst

class Drivetrain : public frc2::SubsystemBase {
 public:
  Drivetrain();

  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed,
             units::radians_per_second_t rot,
             bool fieldRelative,
             units::second_t period);

  void UpdateOdometry();

  // Must stay consistent with DriveConst::kMaxModuleSpeed
  static constexpr auto kMaxSpeed = 4.5_mps;
  static constexpr units::radians_per_second_t kMaxAngularSpeed{std::numbers::pi};  // 1/2 rotation per second

  /** Will be called periodically whenever the CommandScheduler runs. */
  void Periodic() override;

 private:
  // Module locations (relative to robot center)
  frc::Translation2d m_frontLeftLocation{+0.381_m, +0.381_m};
  frc::Translation2d m_frontRightLocation{+0.381_m, -0.381_m};
  frc::Translation2d m_backLeftLocation{-0.381_m, +0.381_m};
  frc::Translation2d m_backRightLocation{-0.381_m, -0.381_m};

  // -----------------------------
  // Swerve modules (PRIMARY CTOR)
  // -----------------------------
  // SwerveModule(int driveCAN, int turnCAN, int cancoderId,
  //              units::radian_t angleOffset, units::meters_per_second_t maxSpeed,
  //              const std::string& name)

  SwerveModule m_frontLeft{
      DriveIds::kFL_Drive, DriveIds::kFL_Turn, DriveIds::kFL_CANCoder,
      DriveConst::kFL_Offset, DriveConst::kMaxModuleSpeed, "FL"};

  SwerveModule m_frontRight{
      DriveIds::kFR_Drive, DriveIds::kFR_Turn, DriveIds::kFR_CANCoder,
      DriveConst::kFR_Offset, DriveConst::kMaxModuleSpeed, "FR"};

  SwerveModule m_backLeft{
      DriveIds::kBL_Drive, DriveIds::kBL_Turn, DriveIds::kBL_CANCoder,
      DriveConst::kBL_Offset, DriveConst::kMaxModuleSpeed, "BL"};

  SwerveModule m_backRight{
      DriveIds::kBR_Drive, DriveIds::kBR_Turn, DriveIds::kBR_CANCoder,
      DriveConst::kBR_Offset, DriveConst::kMaxModuleSpeed, "BR"};

  // Gyro (still AnalogGyro for now)
  frc::AnalogGyro m_gyro{0};

  frc::SwerveDriveKinematics<4> m_kinematics{
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation};

  // Pose estimator (example gains; tune for your robot)
  frc::SwerveDrivePoseEstimator<4> m_poseEstimator{
      m_kinematics,
      frc::Rotation2d{},
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_backLeft.GetPosition(), m_backRight.GetPosition()},
      frc::Pose2d{},
      {0.1, 0.1, 0.1},   // state std devs (x, y, theta)
      {0.1, 0.1, 0.1}    // vision std devs (x, y, theta)
  };

  // Drivetrain network table
  std::shared_ptr<nt::NetworkTable> DrivetrainNetTable =
      nt::NetworkTableInstance::GetDefault().GetTable("2227/Drivetrain");
};
