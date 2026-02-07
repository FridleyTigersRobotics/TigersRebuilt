#pragma once

// --- WPILib / vendor includes ---
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>

#include <units/time.h>

#include <Eigen/Core>
#include <studica/AHRS.h>  // navX

// --- Project includes ---
#include "Constants.h"
#include "Vision.h"

/**
 * VisionPoseEstimator
 *
 * A lightweight façade to provide a “drop-in” vision pose to your drivetrain’s
 * SwerveDrivePoseEstimator update loop. It:
 *  - Reads the navX pitch/roll to gate vision trust when the robot is tipped.
 *  - Computes per-measurement standard deviations via Vision (PhotonVision wrapper).
 *  - Caches last pose, timestamp, and std-devs for the drivetrain to consume.
 *  - Publishes NetworkTables telemetry (targets/used/ignored, pitch/roll, tilt scale).
 *
 * Usage:
 *   - In Drivetrain ctor: VisionPoseEstimator::SetNavX(&m_gyro);
 *                         VisionPoseEstimator::SetVision(&m_vision);
 *   - Each loop:          auto pose = VisionPoseEstimator::GetEstimatedGlobalPose(odometryPose);
 *                         estimator.AddVisionMeasurement(pose, GetLastTimestamp(), stdDevsArray);
 */
class VisionPoseEstimator {
 public:
  // --------------------------------------------------------------------------
  // Configuration (call once at robot init)
  // --------------------------------------------------------------------------

  /** Bind the Vision subsystem (PhotonVision wrapper). */
  static void SetVision(Vision* vision) { m_vision = vision; }

  /** Bind the navX (for pitch/roll tilt gating). */
  static void SetNavX(studica::AHRS* navx) { m_navx = navx; }

  // --------------------------------------------------------------------------
  // Main API (call every loop from drivetrain)
  // --------------------------------------------------------------------------

  /**
   * Returns the latest camera-based robot pose if available; otherwise returns
   * the provided odometry pose. Also updates timestamp/std-devs caches.
   *
   * @param currentPose           Current odometry/estimator pose.
   * @param tiltThresholdDegrees  Max |pitch| or |roll| before vision is de-weighted.
   */
static frc::Pose2d GetEstimatedGlobalPose(
    const frc::Pose2d& currentPose,
    double tiltThresholdDegrees = constants::Vision::kTiltThresholdDegrees) {
  currentOdometryPose = currentPose;

  // Read/publish navX tilt once so entries always update
  double pitchDeg = 0.0;
  double rollDeg = 0.0;
  if (m_navx) {
    pitchDeg = m_navx->GetPitch(); // degrees
    rollDeg = m_navx->GetRoll();   // degrees
  }
  VisionNetTable->PutNumber("RobotPitch", pitchDeg);
  VisionNetTable->PutNumber("RobotRoll", rollDeg);

  // Default telemetry (updated below if vision succeeds)
  VisionNetTable->PutNumber("VisionTiltScale", 1.0);

  const units::second_t now{frc::Timer::GetFPGATimestamp()};

  // Try to form a vision estimate
  if (m_vision) {
    auto result = m_vision->GetLatestResult(); // your Vision wrapper already avoids stale cache
    if (result.HasTargets()) {
      auto& estimator = m_vision->GetEstimator();
      auto visionEst = estimator.EstimateCoprocMultiTagPose(result);
      if (!visionEst) {
        visionEst = estimator.EstimateLowestAmbiguityPose(result);
      }
      if (visionEst) {
        const units::second_t newTs{visionEst->timestamp};

        // Freshness guard (~250 ms)
        if ((now - newTs) > constants::Vision::msStaleCam) {
          VisionNetTable->PutBoolean("VisionHasTargets", true);
          VisionNetTable->PutBoolean("VisionIgnoredTilt", false);
          VisionNetTable->PutBoolean("VisionUsed", false);
          return currentOdometryPose; // no timestamp advance
        }

        // Duplicate/out-of-order guard
        if (newTs <= lastTimestamp) {
          VisionNetTable->PutBoolean("VisionHasTargets", true);
          VisionNetTable->PutBoolean("VisionIgnoredTilt", false);
          VisionNetTable->PutBoolean("VisionUsed", false);
          return currentOdometryPose; // no timestamp advance
        }

        // Accept: cache pose & timestamp
        lastPose = visionEst->estimatedPose.ToPose2d();
        lastTimestamp = newTs;

        // Base stddevs from Vision heuristic
        lastStdDevs = m_vision->GetEstimationStdDevs(lastPose);

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
    }
  }

  // Fallback: use odometry for this loop WITHOUT advancing lastTimestamp
  lastPose = currentPose;
  lastStdDevs = Eigen::Matrix<double, 3, 1>{1.0, 1.0, 1.0};
  VisionNetTable->PutBoolean("VisionHasTargets", false);
  VisionNetTable->PutBoolean("VisionIgnoredTilt", false);
  VisionNetTable->PutBoolean("VisionUsed", false);
  VisionNetTable->PutNumber("VisionTiltScale", 1.0);
  return lastPose;
}

  // --------------------------------------------------------------------------
  // Accessors for drivetrain
  // --------------------------------------------------------------------------

  /** Timestamp (seconds) for the last vision measurement. */
  static units::second_t GetLastTimestamp() { return lastTimestamp; }

  /** Std-devs for the last vision measurement: [x(m), y(m), theta(rad)]. */
  static Eigen::Matrix<double, 3, 1> GetLastStdDevs() { return lastStdDevs; }

 private:
  // --------------------------------------------------------------------------
  // Bound devices / subsystems
  // --------------------------------------------------------------------------
  static inline Vision*        m_vision = nullptr;
  static inline studica::AHRS* m_navx   = nullptr;

  // --------------------------------------------------------------------------
  // Cached data for the drivetrain
  // --------------------------------------------------------------------------
  static inline frc::Pose2d          currentOdometryPose{};
  static inline frc::Pose2d          lastPose{};
  static inline units::second_t      lastTimestamp{0_s};
  static inline Eigen::Matrix<double, 3, 1> lastStdDevs{};

  // NetworkTables path for vision telemetry
  static inline std::shared_ptr<nt::NetworkTable> VisionNetTable =
      nt::NetworkTableInstance::GetDefault().GetTable("2227/Vision");
};