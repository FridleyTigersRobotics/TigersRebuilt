/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
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

    const auto& targets = m_latestResult.GetTargets();
    if (targets.empty()) {
      // No tags visible -> extremely uncertain
      estStdDevs << std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max();
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
      estStdDevs << std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max();
      return estStdDevs;
    }

    avgDist /= numTags;

    // Simple heuristic: uncertainty grows with distance, shrinks with number of tags
    double xyStd = std::pow(avgDist.value(), 1.1) / numTags;  // meters
    double rotStd = std::pow(avgDist.value(), 1.2) / numTags; // radians

    // Clamp to avoid zero uncertainty
    xyStd = std::max(xyStd, 0.01);
    rotStd = std::max(rotStd, 0.01);

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
