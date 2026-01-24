
/*
 * MIT License
 * (unchanged header)
 */

#pragma once

#include <functional>
#include <memory>
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
#include <numbers>   // for std::numbers::pi

#include "Constants.h"

class Vision {
 public:
  /**
   * @param estConsumer Lambda that will accept a pose estimate and pass it to
   * your desired SwerveDrivePoseEstimator.
   */
  Vision(std::function<void(frc::Pose2d, units::second_t,
                            Eigen::Matrix<double, 3, 1>)>
             estConsumer)
      : estConsumer{estConsumer} {}

  photon::PhotonPipelineResult GetLatestResult() { return m_latestResult; }

  void Periodic() {
    for (const auto& result : camera.GetAllUnreadResults()) {
      m_latestResult = result;

      auto visionEst = photonEstimator.EstimateCoprocMultiTagPose(result);
      if (!visionEst) {
        visionEst = photonEstimator.EstimateLowestAmbiguityPose(result);
      }

      if (visionEst) {
        estConsumer(visionEst->estimatedPose.ToPose2d(), visionEst->timestamp,
                    ComputeAutoStdDevs(visionEst->estimatedPose.ToPose2d()));
      }
    }
  }

  // Computes standard deviations automatically from visible tags and distance
  Eigen::Matrix<double, 3, 1> ComputeAutoStdDevs(frc::Pose2d estimatedPose) {
    Eigen::Matrix<double, 3, 1> estStdDevs;

    // Use large-but-finite "ignore" values to avoid numerical issues.
    constexpr double kBigXY = 10.0;                 // meters
    constexpr double kBigTheta = std::numbers::pi;  // radians
      // or constexpr double kBigTheta = std::numbers::pi_v<double>;


    const auto& targets = m_latestResult.GetTargets();
    if (targets.empty()) {
      // No tags visible -> very uncertain but finite
      estStdDevs << kBigXY, kBigXY, kBigTheta;
      return estStdDevs;
    }

    units::meter_t avgDist = 0_m;
    int numTags = 0;

    for (const auto& tgt : targets) {
      auto tagPose = photonEstimator.GetFieldLayout().GetTagPose(tgt.GetFiducialId());
      if (tagPose) {
        numTags++;
        avgDist += tagPose->ToPose2d().Translation().Distance(
            estimatedPose.Translation());
      }
    }

    if (numTags == 0) {
      estStdDevs << kBigXY, kBigXY, kBigTheta;
      return estStdDevs;
    }

    avgDist /= numTags;

    // Simple heuristic: uncertainty grows with distance, shrinks with number of tags
    double xyStd = std::pow(avgDist.value(), 1.1) / numTags;  // meters
    double rotStd = std::pow(avgDist.value(), 1.2) / numTags; // radians

    // Clamp to avoid zero uncertainty
    xyStd = std::max(xyStd, 0.01);
    rotStd = std::max(rotStd, 0.01);

    // Also clamp to finite upper bounds
    xyStd = std::min(xyStd, kBigXY);
    rotStd = std::min(rotStd, kBigTheta);

    estStdDevs << xyStd, xyStd, rotStd;
    return estStdDevs;
  }

  // Legacy wrapper for backward compatibility
  Eigen::Matrix<double, 3, 1> GetEstimationStdDevs(frc::Pose2d pose) {
    return ComputeAutoStdDevs(pose);
  }

  photon::PhotonPoseEstimator& GetEstimator() { return photonEstimator; }

 private:
  photon::PhotonPoseEstimator photonEstimator{constants::Vision::kTagLayout,
                                              constants::Vision::kRobotToCam};
  photon::PhotonCamera camera{constants::Vision::kCameraName};
  photon::PhotonPipelineResult m_latestResult;
  std::function<void(frc::Pose2d, units::second_t, Eigen::Matrix<double, 3, 1>)>
      estConsumer;
};
