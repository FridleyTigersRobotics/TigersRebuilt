
#include "subsystems/Drivetrain.h"

#include <cmath>
#include <frc/Timer.h>
#include <units/time.h>
#include <units/angle.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/VisionPoseEstimator.h"

using namespace pathplanner;

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
  const auto yaw_deg = frc::Rotation2d{units::degree_t{m_gyro.GetAngle()}};
  return frc::Rotation2d{ -yaw_deg }; // <— invert once, use everywhere
}

// -------------------------
// Lifecycle / periodic
// -------------------------
Drivetrain::Drivetrain()
    // Initialize navX with SPI com type and slew-rate limiter rates (scalar/second)
    : m_gyro{studica::AHRS::NavXComType::kMXP_SPI},
      m_xLimiter{4.0 / units::second_t{1.0}},
      m_yLimiter{4.0 / units::second_t{1.0}},
      m_rotLimiter{5.0 / units::second_t{1.0}} {
  m_gyro.ZeroYaw();
  ConfigureAutoBuilder();
  frc::SmartDashboard::PutData("Field",&m_field);
}

void Drivetrain::Periodic() {
  UpdateOdometry();
  
  //Get current pose from the SwerveDrivePoseEstimator
  const auto pose = m_poseEstimator.GetEstimatedPosition();
  DrivetrainNetTable->PutNumber("Robot X (m)", pose.X().to<double>());
  DrivetrainNetTable->PutNumber("Robot Y (m)", pose.Y().to<double>());
  DrivetrainNetTable->PutNumber("Robot Heading (deg)", pose.Rotation().Degrees().to<double>());

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
  frc::Rotation2d rotationvalue = GetGyroRotation();
  double rotationdegrees = rotationvalue.Degrees().value();
  DrivetrainNetTable->PutNumber("Gyro Angle (deg, continuous)", rotationdegrees);
  m_field.SetRobotPose(Drivetrain::getPose());
}

void Drivetrain::ZeroGyro() { m_gyro.ZeroYaw(); }

void Drivetrain::SetXStance(){
  
  const units::meters_per_second_t zero{0.0};
 
  frc::SwerveModuleState fl{zero, frc::Rotation2d{units::degree_t{+45}}};
  frc::SwerveModuleState fr{zero, frc::Rotation2d{units::degree_t{-45}}};
  frc::SwerveModuleState bl{zero, frc::Rotation2d{units::degree_t{-45}}};
  frc::SwerveModuleState br{zero, frc::Rotation2d{units::degree_t{+45}}};

  m_frontLeft .SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft  .SetDesiredState(bl);
  m_backRight .SetDesiredState(br);

}

frc2::CommandPtr Drivetrain::cmdSetXStance(){
  return RunOnce([this] {SetXStance();});
}

frc::Pose2d Drivetrain::getPose(){
  return m_poseEstimator.GetEstimatedPosition();
}

void Drivetrain::resetPose(frc::Pose2d poseinput){
  m_poseEstimator.ResetPose(poseinput);
}

frc::ChassisSpeeds Drivetrain::getRobotRelativeSpeeds(){
  //Read *measured* module states
  frc::SwerveModuleState fl = m_frontLeft.GetState();   // speed m/s, angle Rotation2d
  frc::SwerveModuleState fr = m_frontRight.GetState();
  frc::SwerveModuleState bl = m_backLeft.GetState();
  frc::SwerveModuleState br = m_backRight.GetState();
  return m_kinematics.ToChassisSpeeds(fl, fr, bl, br);
}

//PathPlanner
void Drivetrain::ConfigureAutoBuilder(){
  RobotConfig config = RobotConfig::fromGUISettings();
  // Configure the AutoBuilder last
  AutoBuilder::configure(
      [this](){ return getPose(); }, // Robot pose supplier
      [this](frc::Pose2d pose){ resetPose(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
      [this](){ return getRobotRelativeSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      [this](auto speeds, auto feedforwards){ Drive(speeds.vx, speeds.vy, speeds.omega, false, units::millisecond_t{20}); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
      std::make_shared<PPHolonomicDriveController>( // PPHolonomicController is the built in path following controller for holonomic drive trains
          PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
          PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
      ),
      config, // The robot configuration
      []() {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          auto alliance = frc::DriverStation::GetAlliance();
          if (alliance) {
              return alliance.value() == frc::DriverStation::Alliance::kRed;
          }
          return false;
      },
      this // Reference to this subsystem to set requirements
  );
}