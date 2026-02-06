#pragma once

#include <functional>
#include <cmath>
#include <limits>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/targeting/PhotonPipelineResult.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <units/time.h>
#include <units/length.h>
#include <numbers>

#include "Constants.h"

/**
 * Vision
 *
 * - Provides direct access to the newest PhotonVision frame (cached fallback).
 * - Optional Periodic() path can push pose estimates to a consumer callback.
 * - Supplies a heuristic for per-measurement standard deviations.
 */
class Vision {
 public:
  using EstConsumer =
      std::function<void(frc::Pose2d, units::second_t, Eigen::Matrix<double, 3, 1>)>;

  explicit Vision(EstConsumer estConsumer) : estConsumer{estConsumer} {}

  // Return newest unread frame if available; otherwise return cached result.
  photon::PhotonPipelineResult GetLatestResult() {
    auto unread = camera.GetAllUnreadResults();      // recommended API
    if (!unread.empty()) {
      m_latestResult = unread.back();                // newest frame
    }
    return m_latestResult;                           // cached fallback
  }

  // Optional: produce pose estimates for each unread frame.
  void Periodic() {
    for (const auto& result : camera.GetAllUnreadResults()) {
      m_latestResult = result;
      auto visionEst = photonEstimator.EstimateCoprocMultiTagPose(result);
      if (!visionEst) {
        visionEst = photonEstimator.EstimateLowestAmbiguityPose(result);
      }
      if (visionEst) {
        estConsumer(visionEst->estimatedPose.ToPose2d(),
                    visionEst->timestamp,
                    ComputeAutoStdDevs(visionEst->estimatedPose.ToPose2d()));
      }
    }
  }

  // Compute [σx (m), σy (m), σθ (rad)] from visible tags and average distance.
  Eigen::Matrix<double, 3, 1> ComputeAutoStdDevs(frc::Pose2d estimatedPose) {
    Eigen::Matrix<double, 3, 1> estStdDevs;

    constexpr double kBigXY = 10.0;                 // meters (finite "very large")
    constexpr double kBigTheta = std::numbers::pi;  // radians

    const auto& targets = m_latestResult.GetTargets();
    if (targets.empty()) {
      estStdDevs << kBigXY, kBigXY, kBigTheta;
      return estStdDevs;
    }

    units::meter_t avgDist = 0_m;
    int numTags = 0;
    for (const auto& tgt : targets) {
      auto tagPose =
          photonEstimator.GetFieldLayout().GetTagPose(tgt.GetFiducialId());
      if (tagPose) {
        numTags++;
        avgDist += tagPose->ToPose2d()
                       .Translation()
                       .Distance(estimatedPose.Translation());
      }
    }

    if (numTags == 0) {
      estStdDevs << kBigXY, kBigXY, kBigTheta;
      return estStdDevs;
    }

    avgDist /= numTags;

    // Heuristic scaling: increase with distance, decrease with number of tags.
    double xyStd = std::pow(avgDist.value(), 1.1) / numTags;
    double rotStd = std::pow(avgDist.value(), 1.2) / numTags;

    // Clamp to stable, nonzero ranges.
    xyStd = std::clamp(xyStd, 0.01, kBigXY);
    rotStd = std::clamp(rotStd, 0.01, kBigTheta);

    estStdDevs << xyStd, xyStd, rotStd;
    return estStdDevs;
  }

  // Provided for callers that still use this naming.
  Eigen::Matrix<double, 3, 1> GetEstimationStdDevs(frc::Pose2d pose) {
    return ComputeAutoStdDevs(pose);
  }

  photon::PhotonPoseEstimator& GetEstimator() { return photonEstimator; }

 private:
  photon::PhotonPoseEstimator photonEstimator{
      constants::Vision::kTagLayout,        // field layout
      constants::Vision::kRobotToCam        // robot->camera transform
  };                                        // [2](https://cummins365-my.sharepoint.com/personal/kb895_cummins_com/Documents/Microsoft%20Copilot%20Chat%20Files/Constants.h)

  photon::PhotonCamera camera{constants::Vision::kCameraName}; // must match PV name
                                                               // [2](https://cummins365-my.sharepoint.com/personal/kb895_cummins_com/Documents/Microsoft%20Copilot%20Chat%20Files/Constants.h)
  photon::PhotonPipelineResult m_latestResult{};
  EstConsumer estConsumer;
}
;