
#pragma once

#include <frc2/command/SubsystemBase.h>
#include <numbers>

// navX (Studica vendor lib) + SPI port enum
#include <studica/AHRS.h>
#include <frc/SPI.h>

#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/XboxController.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

// Units headers
#include <units/angle.h>
#include <units/velocity.h>
#include <units/time.h>
#include <units/dimensionless.h>  // scalar, scalar_t

#include "SwerveModule.h"

namespace DriveIds {
constexpr int kFL_Drive = 4;
constexpr int kFL_Turn  = 5;
constexpr int kFR_Drive = 3;
constexpr int kFR_Turn  = 2;
constexpr int kBL_Drive = 8;
constexpr int kBL_Turn  = 9;
constexpr int kBR_Drive = 7;
constexpr int kBR_Turn  = 6;

constexpr int kFL_CANCoder = 11;
constexpr int kFR_CANCoder = 13;
constexpr int kBL_CANCoder = 10;
constexpr int kBR_CANCoder = 12;
}  // namespace DriveIds

namespace DriveConst {
constexpr auto kMaxModuleSpeed = 4.5_mps;

constexpr auto kFL_Offset = -1.404_rad;
constexpr auto kFR_Offset = -2.315_rad;
constexpr auto kBL_Offset = -2.331_rad;
constexpr auto kBR_Offset = -0.205_rad;
}  // namespace DriveConst

class Drivetrain : public frc2::SubsystemBase {
 public:
  Drivetrain();

  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed,
             units::radians_per_second_t rot,
             bool fieldRelative,
             units::second_t period);

  // Xbox drive with deadband + cubic shaping + slew-rate limiting
  void DriveFromXbox(const frc::XboxController& controller,
                     bool fieldRelative,
                     units::second_t period,
                     double deadband = 0.05);

  void UpdateOdometry();

  static constexpr auto kMaxSpeed = 4.5_mps;
  static constexpr units::radians_per_second_t kMaxAngularSpeed{12.0};  // 12 rad/s

  /*
    Calculated max angular speed:
    r_eff is the distance from the robot center to a module, using r_eff = sqrt(x^2 + y^2),
    where x and y are the module's Translation2d coordinates in meters.

    The theoretical maximum angular velocity is:
    omega_max ≈ v_max / r_eff
    where v_max is the robot's maximum linear wheel speed.

    Choose a final kMaxAngularSpeed slightly below the theoretical value
    (about 90–95%) to provide margin for voltage sag, friction, and real-world losses.
  */

  void Periodic() override;

  void ZeroGyro();

 private:
  // Module locations (relative to robot center)
  frc::Translation2d m_frontLeftLocation{+0.260_m, +0.260_m};
  frc::Translation2d m_frontRightLocation{+0.260_m, -0.260_m};
  frc::Translation2d m_backLeftLocation{-0.260_m, +0.260_m};
  frc::Translation2d m_backRightLocation{-0.260_m, -0.260_m};

  // Swerve modules
  SwerveModule m_frontLeft{DriveIds::kFL_Drive, DriveIds::kFL_Turn, DriveIds::kFL_CANCoder,
                           DriveConst::kFL_Offset, DriveConst::kMaxModuleSpeed, "FL"};
  SwerveModule m_frontRight{DriveIds::kFR_Drive, DriveIds::kFR_Turn, DriveIds::kFR_CANCoder,
                            DriveConst::kFR_Offset, DriveConst::kMaxModuleSpeed, "FR"};
  SwerveModule m_backLeft{DriveIds::kBL_Drive, DriveIds::kBL_Turn, DriveIds::kBL_CANCoder,
                          DriveConst::kBL_Offset, DriveConst::kMaxModuleSpeed, "BL"};
  SwerveModule m_backRight{DriveIds::kBR_Drive, DriveIds::kBR_Turn, DriveIds::kBR_CANCoder,
                           DriveConst::kBR_Offset, DriveConst::kMaxModuleSpeed, "BR"};

  // navX (Studica) — constructed in .cpp with com type
  studica::AHRS m_gyro;

  frc::SwerveDriveKinematics<4> m_kinematics{
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation};

  frc::SwerveDrivePoseEstimator<4> m_poseEstimator{
      m_kinematics,
      frc::Rotation2d{},  // initial heading
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_backLeft.GetPosition(), m_backRight.GetPosition()},
      frc::Pose2d{},
      {0.1, 0.1, 0.1},
      {0.1, 0.1, 0.1}};

  std::shared_ptr<nt::NetworkTable> DrivetrainNetTable =
      nt::NetworkTableInstance::GetDefault().GetTable("2227/Drivetrain");

  // Slew-rate limiters (template param is a UNIT TAG; rates set in .cpp)
  frc::SlewRateLimiter<units::scalar> m_xLimiter;
  frc::SlewRateLimiter<units::scalar> m_yLimiter;
  frc::SlewRateLimiter<units::scalar> m_rotLimiter;

  static double ShapeInput(double v, double deadband);
  frc::Rotation2d GetGyroRotation();  // non-const (AHRS getters are non-const)
};
