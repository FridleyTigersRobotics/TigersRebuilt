#pragma once
// --- WPILib / vendor includes ---
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>
#include <units/time.h>
#include <Eigen/Core>
#include <studica/AHRS.h> // navX
// --- Project includes ---
#include "Constants.h"
#include "Vision.h"

/**
 * VisionPoseEstimator
 *
 * - NO global pose solve: derive Field→Robot directly from one observed tag.
 *   Field→Robot = Field→Tag · Tag→Camera · Camera→Robot
 * - Reads navX pitch/roll to de-weight vision when tipped.
 * - Computes per-measurement stddevs via Vision (same cached frame this loop).
 * - Caches last pose, timestamp, and stddevs for the drivetrain to consume.
 * - Publishes NetworkTables telemetry (targets/used/ignored, pitch/roll, tilt scale).
 */
class VisionPoseEstimator {
 public:
  // ---------------------- Configuration (call once at robot init) ----------------------
  static void SetVision(Vision* vision) { m_vision = vision; }     // single-drain via BeginFrame/PeekFrame
  static void SetNavX(studica::AHRS* navx) { m_navx = navx; }

  // ---------------------- Pose conversion helper (usable from static methods) ---------
  static inline frc::Pose3d Pose2dTo3d(const frc::Pose2d& p2d, units::meter_t z = 0_m) {
    return frc::Pose3d{
        frc::Translation3d{p2d.X(), p2d.Y(), z},
        frc::Rotation3d{0_rad, 0_rad, p2d.Rotation().Radians()}};
  }

  // ---------------------- Main API (call every loop from drivetrain) ------------------
  static frc::Pose2d GetEstimatedGlobalPose(
      const frc::Pose2d& currentPose,
      double tiltThresholdDegrees = constants::Vision::kTiltThresholdDegrees) {
    currentOdometryPose = currentPose;

    // navX tilt
    double pitchDeg = 0.0;
    double rollDeg  = 0.0;
    if (m_navx) {
      pitchDeg = m_navx->GetPitch();
      rollDeg  = m_navx->GetRoll();
    }
    VisionNetTable->PutNumber("RobotPitch", pitchDeg);
    VisionNetTable->PutNumber("RobotRoll",  rollDeg);
    VisionNetTable->PutNumber("VisionTiltScale", 1.0);

    const units::second_t now{frc::Timer::GetFPGATimestamp()};

    // Pull unread frames ONCE and share that same frame for pose + stddevs
    if (m_vision) {
      m_vision->BeginFrame();                                // single-drain for this loop
      const photon::PhotonPipelineResult* pres = m_vision->PeekFrame();  // same frame

      // Quick visibility telemetry (kept version-agnostic)
      VisionNetTable->PutBoolean("ResultHasTargets", pres && pres->HasTargets());
      VisionNetTable->PutNumber ("ResultNumTargets", pres ? pres->GetTargets().size() : 0);

      if (pres && pres->HasTargets()) {
        // --- Pick target (lowest ambiguity, fallback largest area) ---
          const auto& targets = pres->GetTargets();
          const photon::PhotonTrackedTarget* best = nullptr;

          double bestAmb = std::numeric_limits<double>::infinity();
          for (const auto& t : targets) {
            if (t.GetFiducialId() <= 0) continue;
            const double amb = t.GetPoseAmbiguity(); // -1 if not provided
            if (amb >= 0.0 && amb < bestAmb) { bestAmb = amb; best = &t; }
          }
          if (best == nullptr) {
            double maxArea = -1.0;
            for (const auto& t : targets) {
              if (t.GetFiducialId() <= 0) continue;
              if (t.GetArea() > maxArea) { maxArea = t.GetArea(); best = &t; }
            }
          }
          if (best == nullptr) {
            VisionNetTable->PutBoolean("VisionHasTargets", true);
            VisionNetTable->PutBoolean("VisionIgnoredTilt", false);
            VisionNetTable->PutBoolean("VisionUsed", false);
            return currentOdometryPose;
          }

          // Field→Tag from layout (no global solve)
          auto& estimator = m_vision->GetEstimator();            // only to access layout
          const auto& layout = estimator.GetFieldLayout();
          auto tagPoseOpt = layout.GetTagPose(best->GetFiducialId());
          if (!tagPoseOpt) {
            VisionNetTable->PutBoolean("VisionHasTargets", true);
            VisionNetTable->PutBoolean("VisionIgnoredTilt", false);
            VisionNetTable->PutBoolean("VisionUsed", false);
            return currentOdometryPose;
          }
          const frc::Pose3d fieldToTag = *tagPoseOpt;

          // Measured per-frame transform + fixed mount
          const frc::Transform3d camToTag   = best->GetBestCameraToTarget(); // PV build-dependent direction
          const frc::Transform3d tagToCam   = camToTag.Inverse();
          //const frc::Transform3d robotToCam = constants::Vision::kRobotToCam;      // MUST be Robot→Camera
          frc::Transform3d robotToCam =
              m_robotToCamSupplier ? m_robotToCamSupplier()
                                  : constants::Vision::kRobotToCam;
          const frc::Transform3d camToRobot = robotToCam.Inverse();                 // Camera→Robot

          // Telemetry: reveal your mount immediately (common root cause if zero)
          VisionNetTable->PutNumber("VT_RobotToCam_X_m", robotToCam.X().to<double>());
          VisionNetTable->PutNumber("VT_RobotToCam_Y_m", robotToCam.Y().to<double>());
          VisionNetTable->PutNumber("VT_RobotToCam_Z_m", robotToCam.Z().to<double>());

          // Compose both candidates, choose plausible one
          // A (expected): Field→Robot = Field→Tag · Tag→Camera · Camera→Robot
          const frc::Pose3d fieldToRobot_A = fieldToTag.TransformBy(tagToCam).TransformBy(camToRobot);
          // B (alternate): Field→Robot = Field→Tag · Camera→Tag · Robot→Camera
          const frc::Pose3d fieldToRobot_B = fieldToTag.TransformBy(camToTag).TransformBy(robotToCam);

          // Diagnostics: distances to tag and camera-tag range
          const auto t_ct = camToTag.Translation();
          const double camTagRange_m = std::sqrt(
              std::pow(t_ct.X().to<double>(), 2) +
              std::pow(t_ct.Y().to<double>(), 2) +
              std::pow(t_ct.Z().to<double>(), 2));
          const double distA_toTag_m = fieldToRobot_A.Translation().Distance(fieldToTag.Translation()).to<double>();
          const double distB_toTag_m = fieldToRobot_B.Translation().Distance(fieldToTag.Translation()).to<double>();
          VisionNetTable->PutNumber("VT_CamToTag_Range_m", camTagRange_m);
          VisionNetTable->PutNumber("VT_DistA_RobotToTag_m", distA_toTag_m);
          VisionNetTable->PutNumber("VT_DistB_RobotToTag_m", distB_toTag_m);

          // Selector: expect Robot↔Tag roughly matches Camera↔Tag (within tolerance), and never allow robot ≈ tag center
          auto plausible = [](double robotTag, double expected) {
            const double minSep = 0.25; // 25 cm: reject collapse onto tag
            const double tol    = 0.75; // 75 cm: generous to handle mount offsets + noise
            return (robotTag > minSep) && (std::abs(robotTag - expected) < tol);
          };

          frc::Pose3d fieldToRobot3d;
          bool usedA = false;
          if      (plausible(distA_toTag_m, camTagRange_m)) { fieldToRobot3d = fieldToRobot_A; usedA = true; }
          else if (plausible(distB_toTag_m, camTagRange_m)) { fieldToRobot3d = fieldToRobot_B; usedA = false; }
          else {
            VisionNetTable->PutBoolean("VisionHasTargets", true);
            VisionNetTable->PutBoolean("VisionIgnoredTilt", false);
            VisionNetTable->PutBoolean("VisionUsed", false);
            VisionNetTable->PutString ("VisionChain", "Rejected");
            return currentOdometryPose;
          }
          VisionNetTable->PutString("VisionChain", usedA ? "A(Tag->Cam->Robot)" : "B(Cam->Tag->Robot)");

          // Final 2D pose
          lastPose = fieldToRobot3d.ToPose2d();

          // OPTIONAL (bring-up): keep odometry heading so only XY comes from vision
          // lastPose = frc::Pose2d(lastPose.Translation(), currentOdometryPose.Rotation());

          // Timestamp & stddevs (same cached frame)
          const units::second_t newTs = now; // prefer frame time if PV exposes it
          lastTimestamp = newTs;
          lastStdDevs   = m_vision->ComputeAutoStdDevs(lastPose);

          // Existing tilt scaling (unchanged)
          constexpr double kTiltGain = constants::Vision::kTiltGainPerDeg;
          constexpr double kTiltMax  = constants::Vision::kTiltMaxScale;
          const double maxAbsTilt = std::max(std::abs(pitchDeg), std::abs(rollDeg));
          const bool overTilt = (maxAbsTilt > tiltThresholdDegrees);
          if (overTilt) {
            const double over = maxAbsTilt - tiltThresholdDegrees;
            double scale = 1.0 + kTiltGain * over;
            if (scale > kTiltMax) scale = kTiltMax;
            lastStdDevs *= scale;
            VisionNetTable->PutNumber("VisionTiltScale", scale);
          }

          VisionNetTable->PutBoolean("VisionHasTargets", true);
          VisionNetTable->PutBoolean("VisionIgnoredTilt", overTilt);
          VisionNetTable->PutBoolean("VisionUsed", true);
          VisionNetTable->PutNumber ("PoseTimestampSec", lastTimestamp.value());
          return lastPose;
      }
    }

    // Fallback: use odometry for this loop WITHOUT advancing lastTimestamp
    lastPose    = currentPose;
    lastStdDevs = Eigen::Matrix<double, 3, 1>{1.0, 1.0, 1.0};
    VisionNetTable->PutBoolean("VisionHasTargets", false);
    VisionNetTable->PutBoolean("VisionIgnoredTilt", false);
    VisionNetTable->PutBoolean("VisionUsed", false);
    VisionNetTable->PutNumber("VisionTiltScale", 1.0);
    return lastPose;
  }

  // ---------------------- Accessors for drivetrain ------------------------------------
  static units::second_t GetLastTimestamp() { return lastTimestamp; }
  static Eigen::Matrix<double, 3, 1> GetLastStdDevs() { return lastStdDevs; }
  
  static void SetRobotToCamSupplier(std::function<frc::Transform3d()> supplier) {
    m_robotToCamSupplier = std::move(supplier);
  }


 private:
  // Bound devices / subsystems
  static inline Vision*        m_vision = nullptr; // single-drain handled by Vision (BeginFrame/PeekFrame)
  static inline studica::AHRS* m_navx   = nullptr;
  static inline std::function<frc::Transform3d()> m_robotToCamSupplier = nullptr;

  // Cached data for the drivetrain
  static inline frc::Pose2d            currentOdometryPose{};
  static inline frc::Pose2d            lastPose{};
  static inline units::second_t        lastTimestamp{0_s};
  static inline Eigen::Matrix<double, 3, 1> lastStdDevs{};

  // NetworkTables path for vision telemetry
  static inline std::shared_ptr<nt::NetworkTable> VisionNetTable =
      nt::NetworkTableInstance::GetDefault().GetTable("2227/Vision");
};