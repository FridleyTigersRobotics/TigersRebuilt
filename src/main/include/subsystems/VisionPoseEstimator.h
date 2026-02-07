#pragma once
// --- WPILib / vendor includes ---
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <units/time.h>
#include <Eigen/Core>
#include <studica/AHRS.h> // navX
// --- Project includes ---
#include "Constants.h"
#include "Vision.h"

/**
 * VisionPoseEstimator
 *
 * - Reads navX pitch/roll to gate vision trust when the robot is tipped.
 * - Computes per-measurement stddevs via Vision (PhotonVision wrapper) using the SAME frame as the pose estimate.
 * - Caches last pose, timestamp, and stddevs for the drivetrain to consume.
 * - Publishes NetworkTables telemetry (targets/used/ignored, pitch/roll, tilt scale).
 */
class VisionPoseEstimator {
 public:
  // ---------------------- Configuration (call once at robot init) ----------------------
  static void SetVision(Vision* vision) { m_vision = vision; }  // single-drain via BeginFrame/PeekFrame
  static void SetNavX(studica::AHRS* navx) { m_navx = navx; }

  // ---------------------- Main API (call every loop from drivetrain) -------------------
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
      m_vision->BeginFrame(); // single-drain for this loop 
      const photon::PhotonPipelineResult* pres = m_vision->PeekFrame(); // same frame

      // Quick visibility telemetry (kept version-agnostic)
      VisionNetTable->PutBoolean("ResultHasTargets", pres && pres->HasTargets());
      VisionNetTable->PutNumber("ResultNumTargets", pres ? pres->GetTargets().size() : 0);

      if (pres && pres->HasTargets()) {
        auto& estimator = m_vision->GetEstimator();
        auto visionEst  = estimator.EstimateCoprocMultiTagPose(*pres);
        if (!visionEst) {
          visionEst = estimator.EstimateLowestAmbiguityPose(*pres);
        }
        if (visionEst) {
          const units::second_t newTs{visionEst->timestamp};

          // Freshness (~250 ms) and duplicate/out-of-order guards
          if ((now - newTs) > constants::Vision::msStaleCam || newTs <= lastTimestamp) {
            VisionNetTable->PutBoolean("VisionHasTargets", true);
            VisionNetTable->PutBoolean("VisionIgnoredTilt", false);
            VisionNetTable->PutBoolean("VisionUsed", false);
            return currentOdometryPose; // do NOT advance lastTimestamp
          }

          // Accept: cache pose & timestamp
          lastPose      = visionEst->estimatedPose.ToPose2d();
          lastTimestamp = newTs;

          // (Optional) Publish the pose timestamp actually used for fusion
          VisionNetTable->PutNumber("PoseTimestampSec", lastTimestamp.value());

          // Stddevs computed from SAME frame (via Vision heuristic)
          lastStdDevs = m_vision->ComputeAutoStdDevs(lastPose);  // single-frame stddevs  

          // Smooth tilt scaling
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
          return lastPose;
        }

        // Had targets but couldn't estimate a pose
        VisionNetTable->PutBoolean("VisionHasTargets", true);
        VisionNetTable->PutBoolean("VisionIgnoredTilt", false);
        VisionNetTable->PutBoolean("VisionUsed", false);
        return currentOdometryPose;
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

  // ---------------------- Accessors for drivetrain ----------------------------
  static units::second_t GetLastTimestamp() { return lastTimestamp; }
  static Eigen::Matrix<double, 3, 1> GetLastStdDevs() { return lastStdDevs; }

 private:
  // Bound devices / subsystems
  static inline Vision*        m_vision = nullptr;         // single-drain handled by Vision
  static inline studica::AHRS* m_navx   = nullptr;

  // Cached data for the drivetrain
  static inline frc::Pose2d                 currentOdometryPose{};
  static inline frc::Pose2d                 lastPose{};
  static inline units::second_t             lastTimestamp{0_s};
  static inline Eigen::Matrix<double, 3, 1> lastStdDevs{};

  // NetworkTables path for vision telemetry
  static inline std::shared_ptr<nt::NetworkTable> VisionNetTable =
      nt::NetworkTableInstance::GetDefault().GetTable("2227/Vision");
};