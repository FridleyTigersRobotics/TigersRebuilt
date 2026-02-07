#pragma once

#include <functional>
#include <cmath>
#include <limits>
#include <numbers>

#include <Eigen/Core>

#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>

#include <units/time.h>
#include <units/length.h>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/targeting/PhotonPipelineResult.h>

#include "Constants.h"

/**
 * Vision
 *
 * - Provides the camera's current frame based on unread results (no stale caching).
 * - Optional Periodic() can push pose estimates to a consumer callback.
 * - Provides a heuristic for per-measurement standard deviations.
 */
class Vision {
 public:
  using EstConsumer =
      std::function<void(frc::Pose2d, units::second_t, Eigen::Matrix<double, 3, 1>)>;

  explicit Vision(EstConsumer estConsumer) : estConsumer{estConsumer} {}

  /**
   * Return the newest result from the unread queue if present; otherwise return
   * an empty result (HasTargets=false). This avoids replaying stale frames and
   * avoids deprecated PhotonCamera::GetLatestResult().
   */
  photon::PhotonPipelineResult GetLatestResult() {
    auto unread = camera.GetAllUnreadResults();            // non-deprecated
    if (!unread.empty()) {
      return unread.back();                                // newest frame this loop
    }
    return photon::PhotonPipelineResult{};                 // no new data -> empty
  }

  /**
   * Optional periodic estimator:
   * - Iterates all unread frames this loop.
   * - Forms a pose estimate for each.
   * - Drops estimates older than ~250 ms by comparing the POSE timestamp to FPGA time.
   * - Emits pose + timestamp + auto std-devs to the provided consumer.
   */
  void Periodic() {
    for (const auto& result : camera.GetAllUnreadResults()) {
      auto visionEst = photonEstimator.EstimateCoprocMultiTagPose(result);
      if (!visionEst) {
        visionEst = photonEstimator.EstimateLowestAmbiguityPose(result);
      }
      if (!visionEst) continue;

      // Freshness guard using the estimate's timestamp
      const units::second_t now{frc::Timer::GetFPGATimestamp()};
      const units::second_t ts{units::second_t{visionEst->timestamp}};
      if ((now - ts) > constants::Vision::msStaleCam) continue;

      const frc::Pose2d pose2d = visionEst->estimatedPose.ToPose2d();
      estConsumer(pose2d, ts, ComputeAutoStdDevs(pose2d));
    }
  }

  /**
   * Compute [σx (m), σy (m), σθ (rad)] from visible tags & average distance.
   * Uses the newest unread result if available; otherwise treats as no targets.
   */
  Eigen::Matrix<double, 3, 1> ComputeAutoStdDevs(frc::Pose2d estimatedPose) {
    Eigen::Matrix<double, 3, 1> estStdDevs;
    constexpr double kBigXY = 10.0;
    constexpr double kBigTheta = std::numbers::pi;

    // Pull the newest unread frame, if any (no deprecations or stale cache).
    // We intentionally do NOT fall back to any previously cached result.
    photon::PhotonPipelineResult res{};
    {
      auto unread = camera.GetAllUnreadResults();
      if (!unread.empty()) res = unread.back();
    }

    const auto& targets = res.GetTargets();
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

    xyStd = std::clamp(xyStd, 0.01, kBigXY);
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
};