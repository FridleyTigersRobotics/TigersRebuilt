
#include "subsystems/Drivetrain.h"

#include <cmath>
#include <frc/Timer.h>
#include <units/time.h>
#include <units/angle.h>

using namespace units::literals;  // safe in .cpp (e.g., 20_ms, 1_s)

#include "subsystems/VisionPoseEstimator.h"

// -------------------------
// Helpers
// -------------------------
static inline double ApplyDeadband(double v, double deadband) {
  return (std::abs(v) < deadband) ? 0.0 : v;
}

double Drivetrain::ShapeInput(double v, double deadband) {
  v = ApplyDeadband(v, deadband);
  return v * v * v;  // cubic shaping
}

frc::Rotation2d Drivetrain::GetGyroRotation() {
  // Studica navX returns degrees; method is non-const
  return frc::Rotation2d{units::degree_t{m_gyro.GetAngle()}};
}

// -------------------------
// Lifecycle / periodic
// -------------------------
Drivetrain::Drivetrain()
    // Initialize navX with SPI com type and slew-rate limiter rates (scalar/second)
    : m_gyro{studica::AHRS::NavXComType::kMXP_SPI},
      m_xLimiter{3.0 / 1_s},
      m_yLimiter{3.0 / 1_s},
      m_rotLimiter{4.0 / 1_s} {
  m_gyro.ZeroYaw();
}

void Drivetrain::Periodic() {
  // (optional telemetry)
}

// -------------------------
// Drive APIs
// -------------------------
void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot,
                       bool fieldRelative,
                       units::second_t period) {
  const auto headingForField =
      fieldRelative ? GetGyroRotation() : frc::Rotation2d{0_rad};

  const auto chassis = fieldRelative
      ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, headingForField)
      : frc::ChassisSpeeds{xSpeed, ySpeed, rot};

  auto states = m_kinematics.ToSwerveModuleStates(
      frc::ChassisSpeeds::Discretize(chassis, period));

  m_kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

  auto [fl, fr, bl, br] = states;
  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);
}

void Drivetrain::DriveFromXbox(const frc::XboxController& controller,
                               bool fieldRelative,
                               units::second_t period,
                               double deadband) {
  // WPILib Y is inverted; invert so up is +forward
  const double rawX   = -controller.GetLeftY();
  const double rawY   = -controller.GetLeftX();
  const double rawRot = -controller.GetRightX();

  const double shapedX   = ShapeInput(rawX,   deadband);
  const double shapedY   = ShapeInput(rawY,   deadband);
  const double shapedRot = ShapeInput(rawRot, deadband);

  // 2026 SlewRateLimiter takes/returns units::scalar_t
  const auto limitedX_u   = m_xLimiter.Calculate(units::scalar_t{shapedX});
  const auto limitedY_u   = m_yLimiter.Calculate(units::scalar_t{shapedY});
  const auto limitedRot_u = m_rotLimiter.Calculate(units::scalar_t{shapedRot});

  const double limitedX   = limitedX_u.value();
  const double limitedY   = limitedY_u.value();
  const double limitedRot = limitedRot_u.value();

  const auto xSpeed  = limitedX   * kMaxSpeed;         // meters_per_second_t
  const auto ySpeed  = limitedY   * kMaxSpeed;         // meters_per_second_t
  const auto rotRate = limitedRot * kMaxAngularSpeed;  // radians_per_second_t

  Drive(xSpeed, ySpeed, rotRate, fieldRelative, period);
}

// -------------------------
// Odometry / vision fusion
// -------------------------
void Drivetrain::UpdateOdometry() {
  m_poseEstimator.Update(
      GetGyroRotation(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_backLeft.GetPosition(), m_backRight.GetPosition()});

  const frc::Pose2d visionPose =
      VisionPoseEstimator::GetEstimatedGlobalPose(
          m_poseEstimator.GetEstimatedPosition());

  const units::second_t visionTimestamp =
      VisionPoseEstimator::GetLastTimestamp();

  const Eigen::Matrix<double, 3, 1> stdDevs =
      VisionPoseEstimator::GetLastStdDevs();

  const wpi::array<double, 3> visionStdDevArray{stdDevs(0), stdDevs(1), stdDevs(2)};
  m_poseEstimator.AddVisionMeasurement(visionPose, visionTimestamp, visionStdDevArray);

  // Optional telemetry
  const auto visPose =
      VisionPoseEstimator::GetEstimatedGlobalPose(m_poseEstimator.GetEstimatedPosition());
  DrivetrainNetTable->PutNumber("Vision X", visPose.X().to<double>());
  DrivetrainNetTable->PutNumber("Vision Y", visPose.Y().to<double>());
  DrivetrainNetTable->PutNumber("Vision Heading", visPose.Rotation().Degrees().to<double>());

  DrivetrainNetTable->PutNumber("navX Yaw (deg)", m_gyro.GetYaw());
  DrivetrainNetTable->PutNumber("navX Angle (deg, continuous)", m_gyro.GetAngle());
}

void Drivetrain::ZeroGyro() { m_gyro.ZeroYaw(); }
