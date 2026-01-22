#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/time.h>
#include <Eigen/Core>
#include <studica/AHRS.h>  // navX 2.0
#include "Vision.h"
#include "Constants.h"

/**
 * Replacement for ExampleGlobalMeasurementSensor using navX 2.0.
 * Provides a drop-in API for Drivetrain::UpdateOdometry() using PhotonVision.
 *
 * Tilt handling: If the robot is tipped too far, vision updates are marked
 * with very large std deviations so the pose estimator trusts odometry more.
 * SmartDashboard displays tilt angle and whether vision was ignored.
 */
class ExampleGlobalMeasurementSensor {
 public:
  /** Sets the Vision subsystem to use */
  static void SetVision(Vision* vision) { m_vision = vision; }

  /** Sets the navX AHRS to read pitch/roll from */
  static void SetNavX(studica::AHRS* navx) { m_navx = navx; }

  /**
   * Returns the latest robot pose from the Vision subsystem.
   * If no vision is available, returns current odometry pose.
   *
   * tiltThresholdDegrees: max allowed tilt (pitch or roll) before reducing confidence
   */
  static frc::Pose2d GetEstimatedGlobalPose(const frc::Pose2d& currentPose,
                                            double tiltThresholdDegrees = constants::Vision::tiltThresholdDegrees) {
    currentOdometryPose = currentPose;
    bool visionIgnoredDueToTilt = false;

    if (m_vision) {
      auto result = m_vision->GetLatestResult();
      if (result.HasTargets()) {
        auto& estimator = m_vision->GetEstimator();  // public getter
        auto visionEst = estimator.EstimateCoprocMultiTagPose(result);
        if (!visionEst) {
          visionEst = estimator.EstimateLowestAmbiguityPose(result);
        }

        if (visionEst) {
          // Check robot tilt
          double pitch = 0.0, roll = 0.0;
          if (m_navx) {
            pitch = m_navx->GetPitch();  // degrees
            roll  = m_navx->GetRoll();   // degrees
            if (std::abs(pitch) > tiltThresholdDegrees ||
                std::abs(roll) > tiltThresholdDegrees) {
              visionIgnoredDueToTilt = true;
            }
          }

          lastPose = visionEst->estimatedPose.ToPose2d();
          lastTimestamp = visionEst->timestamp;
          lastStdDevs = m_vision->GetEstimationStdDevs(lastPose);

          if (visionIgnoredDueToTilt) {
            lastStdDevs *= 1000;  // effectively ignore vision
          }

          // SmartDashboard logging
          frc::SmartDashboard::PutBoolean("VisionIgnoredTilt", visionIgnoredDueToTilt);
          frc::SmartDashboard::PutNumber("RobotPitch", pitch);
          frc::SmartDashboard::PutNumber("RobotRoll", roll);

          return lastPose;
        }
      }
    }

    // No vision available → fallback to odometry
    lastPose = currentPose;
    lastTimestamp = frc::Timer::GetFPGATimestamp();
    lastStdDevs = Eigen::Matrix<double, 3, 1>{1.0, 1.0, 1.0};

    // Log odometry fallback
    frc::SmartDashboard::PutBoolean("VisionIgnoredTilt", true);
    frc::SmartDashboard::PutNumber("RobotPitch", m_navx ? m_navx->GetPitch() : 0.0);
    frc::SmartDashboard::PutNumber("RobotRoll",  m_navx ? m_navx->GetRoll()  : 0.0);

    return lastPose;
  }

  /** Returns the timestamp for the last vision measurement */
  static units::second_t GetLastTimestamp() { return lastTimestamp; }

  /** Returns standard deviations for the last vision measurement */
  static Eigen::Matrix<double, 3, 1> GetLastStdDevs() { return lastStdDevs; }

 private:
  static inline Vision* m_vision = nullptr;
  static inline studica::AHRS* m_navx = nullptr;  // navX 2.0
  static inline frc::Pose2d currentOdometryPose{};
  static inline frc::Pose2d lastPose{};
  static inline units::second_t lastTimestamp = 0_s;
  static inline Eigen::Matrix<double, 3, 1> lastStdDevs{};
};
