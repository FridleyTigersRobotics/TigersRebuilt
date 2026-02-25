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
#include <numbers>  // for std::numbers::pi


#include "subsystems/VisionPoseEstimator.h"

using namespace pathplanner;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static inline double ApplyDeadband(double v, double deadband) {
  return (std::abs(v) < deadband) ? 0.0 : v;
}
double Drivetrain::ShapeInput(double v, double deadband) {
  v = ApplyDeadband(v, deadband);
  return v * v * v; // cubic shaping
}
frc::Rotation2d Drivetrain::GetGyroRotation() {
  // Studica navX returns degrees; method is non-const
  const auto yaw_deg = frc::Rotation2d{units::degree_t{m_gyro.GetAngle()}};
  return frc::Rotation2d{ -yaw_deg }; // invert once, use everywhere
}

// ---------------------------------------------------------------------------
// Lifecycle / periodic
// ---------------------------------------------------------------------------
Drivetrain::Drivetrain()
  // Initialize navX with SPI com type and slew-rate limiter rates (scalar/second)
  : m_gyro{studica::AHRS::NavXComType::kMXP_SPI},
    m_vision{nullptr},
    m_xLimiter{4.0 / units::second_t{1.0}},
    m_yLimiter{4.0 / units::second_t{1.0}},
    m_rotLimiter{5.0 / units::second_t{1.0}} {

  m_gyro.ZeroYaw();
  // Bind devices to the vision facade (single-drain occurs inside VisionPoseEstimator)
  VisionPoseEstimator::SetNavX(&m_gyro);
  VisionPoseEstimator::SetVision(&m_vision);  // m_vision is your Vision wrapper instance
  // ^ GetEstimatedGlobalPose() will call Vision::BeginFrame() once per loop. 

  ConfigureAutoBuilder();
  frc::SmartDashboard::PutData("Field",&m_field);

  // Angle is in radians; enable continuous input so error wraps on (-pi, pi]
  m_aimPID.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);

  // Optional: set a small tolerance so AtSetpoint() goes true when close
  m_aimPID.SetTolerance(constants::Driver::kAimTol.value());
}

void Drivetrain::Periodic() {
  UpdateOdometry();
  //Get current pose from the SwerveDrivePoseEstimator
  const auto pose = m_poseEstimator.GetEstimatedPosition();
  DrivetrainNetTable->PutNumber("Robot X (m)", pose.X().to<double>());
  DrivetrainNetTable->PutNumber("Robot Y (m)", pose.Y().to<double>());
  DrivetrainNetTable->PutNumber("Robot Heading (deg)", pose.Rotation().Degrees().to<double>());
}

// ---------------------------------------------------------------------------
// Drive APIs
// ---------------------------------------------------------------------------
void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot,
                       bool fieldRelative,
                       units::second_t period) {
  // Field heading used only for field-relative; your accessor already inverts once consistently
  const auto headingForField =
      fieldRelative ? GetGyroRotation() : frc::Rotation2d{0_rad};

  // ---- Global convention fix: +rot should spin the robot CCW physically ----
  const auto rotCCW = -rot;  // single-point sign flip for angular rate

  // Build chassis speeds (field-relative or robot-relative), then discretize
  const auto chassis = fieldRelative
      ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rotCCW, headingForField)
      : frc::ChassisSpeeds{xSpeed, ySpeed, rotCCW};

  auto states = m_kinematics.ToSwerveModuleStates(
      frc::ChassisSpeeds::Discretize(chassis, period));
  m_kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

  auto [fl, fr, bl, br] = states;
  m_frontLeft .SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft  .SetDesiredState(bl);
  m_backRight .SetDesiredState(br);
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
  const auto limitedX_u   = m_xLimiter .Calculate(units::scalar_t{shapedX});
  const auto limitedY_u   = m_yLimiter .Calculate(units::scalar_t{shapedY});
  const auto limitedRot_u = m_rotLimiter.Calculate(units::scalar_t{shapedRot});

  const double limitedX   = limitedX_u.value();
  const double limitedY   = limitedY_u.value();
  const double limitedRot = limitedRot_u.value();

  const auto xSpeed = limitedX * kMaxSpeed;        // meters_per_second_t
  const auto ySpeed = limitedY * kMaxSpeed;        // meters_per_second_t
  const auto rotRate = limitedRot * kMaxAngularSpeed; // radians_per_second_t
  Drive(xSpeed, ySpeed, rotRate, fieldRelative, period);
}

void Drivetrain::DriveFromXboxAim(const frc::XboxController& controller,
                                  bool fieldRelative,
                                  units::second_t period,
                                  double deadband,
                                  const frc::Translation2d& targetXY) {
  // ----------------- Translation from left stick (unchanged) -----------------
  // WPILib Y is inverted; invert so up is +forward
  const double rawX = -controller.GetLeftY();
  const double rawY = -controller.GetLeftX();

  const double shapedX = ShapeInput(rawX, deadband);
  const double shapedY = ShapeInput(rawY, deadband);

  // 2026 SlewRateLimiter takes/returns units::scalar_t
  const auto limitedX_u = m_xLimiter.Calculate(units::scalar_t{shapedX});
  const auto limitedY_u = m_yLimiter.Calculate(units::scalar_t{shapedY});

  const auto xSpeed = limitedX_u.value() * kMaxSpeed; // m/s
  const auto ySpeed = limitedY_u.value() * kMaxSpeed; // m/s

  // ----------------- Rotation: face the target coordinate (PID) -------------
  const frc::Pose2d pose = getPose();                 // field pose
  const frc::Translation2d delta = targetXY - pose.Translation();

  const frc::Rotation2d targetHeading  = delta.Angle();
  const frc::Rotation2d currentHeading = pose.Rotation();

  // Optional guard: don't spin if we are effectively at the target point
  const bool nearTargetXY = delta.Norm().to<double>() < 0.05; // 5 cm

  // (Optional) manual override on right-stick X — uncomment if desired
  // constexpr double kManualOverrideThresh = 0.15;
  // const double rightX = controller.GetRightX();
  // const bool manualOverride = std::abs(rightX) > kManualOverrideThresh;

  double targetOmega = 0.0; // rad/s

  if (!nearTargetXY /* && !manualOverride */) {
    // PID Calculate(measurement, setpoint) — both in radians
    const double measurement = currentHeading.Radians().value();
    const double setpoint    = targetHeading.Radians().value();

    // If you want to reset I/D when re-entering auto-aim after manual override, call:
    // if (manualOverrideLastLoop && !manualOverride) m_aimPID.Reset();

    double pidOut = m_aimPID.Calculate(measurement, setpoint); // rad/s

    // Clamp to your configured max angular speed
    pidOut = std::clamp(pidOut, -kMaxAngularSpeed.value(), +kMaxAngularSpeed.value());

    targetOmega = pidOut;
  } else {
    // When at the target (or on top of it), hold still and clear I-term
    m_aimPID.Reset();
    targetOmega = 0.0;
  }

  // Apply your rotational slew-rate limiter (normalize -> limit -> denormalize)
  const double targetOmegaNorm = targetOmega / kMaxAngularSpeed.value();
  const auto limitedOmegaNorm_u = m_rotLimiter.Calculate(units::scalar_t{targetOmegaNorm});
  const double limitedOmega = limitedOmegaNorm_u.value() * kMaxAngularSpeed.value();
  const auto rotRate = units::radians_per_second_t{limitedOmega};

  // ----------------- Drive -----------------
  Drive(xSpeed, ySpeed, rotRate, fieldRelative, period);
}

frc2::CommandPtr Drivetrain::cmdAimAtHub(const frc::XboxController& controller,
                                         bool fieldRelative,
                                         units::second_t period,
                                         double deadband) {
  // Capture by VALUE so args are stable inside the command; 'this' is the requirement owner.
  return frc2::cmd::Run(
    [this, &controller, fieldRelative, period, deadband] {
      frc::Translation2d allianceHubCoords;
      const auto alliance = frc::DriverStation::GetAlliance();
      if (alliance && alliance.value() == frc::DriverStation::Alliance::kRed) {
        allianceHubCoords = constants::Field::kRedHubCoord;
      } else {
        allianceHubCoords = constants::Field::kBlueHubCoord;
      }
      DriveFromXboxAim(controller, fieldRelative, period, deadband, allianceHubCoords);
    },
    {this}  // require Drivetrain so default command pauses while this runs
  );
}

// ---------------------------------------------------------------------------
// Odometry / vision fusion
// ---------------------------------------------------------------------------
void Drivetrain::UpdateOdometry() {
  // Update odometry from gyro + wheel module positions
  m_poseEstimator.Update(
      GetGyroRotation(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_backLeft.GetPosition(),  m_backRight.GetPosition()});

  // Get one candidate vision pose (computed relative to a seen tag)
  const frc::Pose2d visionPose =
      VisionPoseEstimator::GetEstimatedGlobalPose(m_poseEstimator.GetEstimatedPosition()); // single-drain inside
  const units::second_t visionTimestamp = VisionPoseEstimator::GetLastTimestamp();
  const Eigen::Matrix<double, 3, 1> stdDevs = VisionPoseEstimator::GetLastStdDevs();
  const wpi::array<double, 3> visionStdDevArray{stdDevs(0), stdDevs(1), stdDevs(2)};

  // Only act when a new vision measurement arrives
  static units::second_t lastUsedTs{0_s};
  const bool newVision = (visionTimestamp > lastUsedTs);

  // --- Snap-or-Nudge thresholds (tune to taste) ---
  constexpr double kSnapDistanceMeters = constants::Vision::kSnapDistanceMeters;
  constexpr double kSnapAngleDeg = constants::Vision::kSnapAngleDeg;

  // Optional: XY-only snap (keep gyro heading) for first-bringup
  constexpr bool   kXYOnlySnap         = false; // set true if you want to preserve yaw on snap

  bool fusedThisLoop = false;

  if (newVision) {
    const frc::Pose2d odomPose = m_poseEstimator.GetEstimatedPosition();
    const double distDelta_m =
        odomPose.Translation().Distance(visionPose.Translation()).to<double>();
    const double headingDelta_deg =
        (odomPose.Rotation() - visionPose.Rotation()).Degrees().to<double>();

    DrivetrainNetTable->PutNumber("Vision_OdomDelta_m", distDelta_m);
    DrivetrainNetTable->PutNumber("Vision_HeadingDelta_deg", headingDelta_deg);

    const bool needSnap =
        (distDelta_m > kSnapDistanceMeters) || (std::abs(headingDelta_deg) > kSnapAngleDeg);

    if (needSnap) {
      // ---- SNAP ----
      if (kXYOnlySnap) {
        // Keep current odometry/gyro heading; snap only X/Y
        const frc::Pose2d xyOnlyVision{visionPose.Translation(), odomPose.Rotation()};
        m_poseEstimator.ResetPose(xyOnlyVision);
      } else {
        // Full pose snap (XY + heading) to the vision-derived pose
        m_poseEstimator.ResetPose(visionPose);
      }
      lastUsedTs = visionTimestamp;
      DrivetrainNetTable->PutBoolean("VisionSnapApplied", true);
      fusedThisLoop = true;
    } else {
      // ---- NUDGE ----
      m_poseEstimator.AddVisionMeasurement(visionPose, visionTimestamp, visionStdDevArray);
      lastUsedTs = visionTimestamp;
      DrivetrainNetTable->PutBoolean("VisionSnapApplied", false);
      fusedThisLoop = true;
    }
  } else {
    DrivetrainNetTable->PutBoolean("VisionSnapApplied", false);
  }

  // Telemetry: vision pose (re-using the same pose we already pulled this loop)
  DrivetrainNetTable->PutNumber("Vision X", visionPose.X().to<double>());
  DrivetrainNetTable->PutNumber("Vision Y", visionPose.Y().to<double>());
  DrivetrainNetTable->PutNumber("Vision Heading", visionPose.Rotation().Degrees().to<double>());
  DrivetrainNetTable->PutBoolean("VisionFusedThisLoop", fusedThisLoop);

  // Extra gyro telemetry (from your existing code)
  DrivetrainNetTable->PutNumber("navX Yaw (deg)", m_gyro.GetYaw());
  frc::Rotation2d rotationvalue = GetGyroRotation();
  double rotationdegrees = rotationvalue.Degrees().value();
  DrivetrainNetTable->PutNumber("Gyro Angle (deg, continuous)", rotationdegrees);

  // Keep the Field2D widget updated
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
  frc::SwerveModuleState fl = m_frontLeft.GetState(); // speed m/s, angle Rotation2d
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
          PIDConstants(constants::Driver::kAutokPt, constants::Driver::kAutokIt, constants::Driver::kAutokDt), // Translation PID constants
          PIDConstants(constants::Driver::kAutokPr, constants::Driver::kAutokIr, constants::Driver::kAutokDr) // Rotation PID constants
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