#pragma once

// --- STL ---
#include <functional>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>   // NEW: per-loop cached frame
#include <algorithm>  // for std::clamp

// --- Eigen / WPILib ---
#include <Eigen/Core>
#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <units/time.h>
#include <units/length.h>
#include <frc/apriltag/AprilTagFieldLayout.h>

// --- PhotonVision ---
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/targeting/PhotonPipelineResult.h>

// --- Project ---
#include "Constants.h"

/**
 * Vision
 *
 * - Provides a per-loop cached camera frame to avoid double-draining unread results.
 * - Optional Periodic() can push pose estimates to a consumer callback (avoid if using pose fusion elsewhere).
 * - Provides a heuristic for per-measurement standard deviations that uses the same cached frame.
 */
class Vision {
 public:
  using EstConsumer =
      std::function<void(frc::Pose2d, units::second_t, Eigen::Matrix<double, 3, 1>)>;

  explicit Vision(EstConsumer estConsumer) : estConsumer{estConsumer} {}

  // --- Per-loop frame management ---------------------------------------------

  /**
   * Pull unread results once and cache the newest for *this* loop.
   * Call this once per robot loop *before* trying to compute vision pose/stddevs.
   */
  void BeginFrame() {
    m_frameThisLoop.reset();
    auto unread = camera.GetAllUnreadResults();  // non-deprecated API
    if (!unread.empty()) {
      m_frameThisLoop = unread.back();           // newest this loop
    }
  }

  /**
   * Peek the frame captured by BeginFrame(); returns nullptr if none this loop.
   */
  const photon::PhotonPipelineResult* PeekFrame() const {
    return m_frameThisLoop ? &*m_frameThisLoop : nullptr;
  }

  /**
   * Compatibility helper: returns newest unread result *or* an empty result.
   * Intentionally does NOT fall back to camera.GetLatestResult() to avoid stale timestamps.
   */
  photon::PhotonPipelineResult GetLatestResult() {
    auto unread = camera.GetAllUnreadResults();
    if (!unread.empty()) {
      return unread.back();
    }
    return photon::PhotonPipelineResult{};  // empty => HasTargets() false
  }

  // --- Optional periodic estimator (avoid if another consumer is draining) ----
  /**
   * Optional periodic estimator:
   * - Iterates all unread frames and emits pose+timestamp+stdDevs to estConsumer.
   * - Drops estimates older than ~250 ms by comparing estimate timestamp to FPGA time.
   *
   * NOTE: If your drivetrain already consumes frames each loop (via VisionPoseEstimator),
   * do NOT also call this, or you'll drain the same unread queue twice.
   */
  void Periodic() {
    for (const auto& result : camera.GetAllUnreadResults()) {
      auto visionEst = photonEstimator.EstimateCoprocMultiTagPose(result);
      if (!visionEst) {
        visionEst = photonEstimator.EstimateLowestAmbiguityPose(result);
      }
      if (!visionEst) continue;

      const units::second_t now{frc::Timer::GetFPGATimestamp()};
      const units::second_t ts{units::second_t{visionEst->timestamp}};
      if ((now - ts) > constants::Vision::msStaleCam) continue;

      const frc::Pose2d pose2d = visionEst->estimatedPose.ToPose2d();
      estConsumer(pose2d, ts, ComputeAutoStdDevs(pose2d));
    }
  }

  // --- Std-dev heuristic ------------------------------------------------------
  /**
   * Compute [σx (m), σy (m), σθ (rad)] based on visible tags & average distance,
   * using the *same* frame cached by BeginFrame() this loop if available.
   * If no frame/targets this loop, returns large stddevs to de-weight vision.
   */
  Eigen::Matrix<double, 3, 1> ComputeAutoStdDevs(frc::Pose2d estimatedPose) {
    Eigen::Matrix<double, 3, 1> estStdDevs;
    constexpr double kBigXY = 10.0;
    constexpr double kBigTheta = std::numbers::pi;

    // Use the SAME frame this loop (no second unread drain)
    const photon::PhotonPipelineResult* pres = PeekFrame();
    if (pres == nullptr) {
      estStdDevs << kBigXY, kBigXY, kBigTheta;
      return estStdDevs;
    }

    const auto& targets = pres->GetTargets();
    if (targets.empty()) {
      estStdDevs << kBigXY, kBigXY, kBigTheta;
      return estStdDevs;
    }

    units::meter_t avgDist = 0_m;
    int numTags = 0;
    for (const auto& tgt : targets) {
      auto tagPose = photonEstimator.GetFieldLayout().GetTagPose(tgt.GetFiducialId());
      if (tagPose) {
        numTags++;
        avgDist += tagPose->ToPose2d().Translation().Distance(estimatedPose.Translation());
      }
    }

    if (numTags == 0) {
      estStdDevs << kBigXY, kBigXY, kBigTheta;
      return estStdDevs;
    }

    avgDist /= numTags;

    double xyStd = std::pow(avgDist.value(), 1.1) / numTags;
    double rotStd = std::pow(avgDist.value(), 1.2) / numTags;

    xyStd  = std::clamp(xyStd,  0.01, kBigXY);
    rotStd = std::clamp(rotStd, 0.01, kBigTheta);

    estStdDevs << xyStd, xyStd, rotStd;
    return estStdDevs;
  }

  // Legacy alias kept for callers that still use the old name.
  Eigen::Matrix<double, 3, 1> GetEstimationStdDevs(frc::Pose2d pose) {
    return ComputeAutoStdDevs(pose);
  }

  photon::PhotonPoseEstimator& GetEstimator() { return photonEstimator; }

 private:
  photon::PhotonPoseEstimator photonEstimator{
      constants::Vision::kTagLayout,
      constants::Vision::kRobotToCam};
  photon::PhotonCamera camera{constants::Vision::kCameraName};

  EstConsumer estConsumer;

  // Per-loop cached frame (valid after BeginFrame() until next loop)
  std::optional<photon::PhotonPipelineResult> m_frameThisLoop;
};