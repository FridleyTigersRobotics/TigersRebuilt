// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>
#include <string_view>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation2d.h>

#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */
namespace constants {

// ============================================================================
// Operator I/O
// ============================================================================
namespace OperatorConstants {
inline constexpr int kDriverControllerPort = 0;
}  // namespace OperatorConstants

// ============================================================================
// Vision (PhotonVision + AprilTags)
// ============================================================================
//
// Coordinate frame (WPILib standard):
//   +X = forward (toward the front bumper)
//   +Y = left
//   +Z = up
//
// Rotation3d order is (roll about X, pitch about Y, yaw about Z)
//   - Pitch DOWN is negative
//   - Yaw LEFT is positive (right is negative)
//
// All distances are meters; all angles are radians.
// ============================================================================
namespace Vision {

// Name that matches the PhotonVision camera (exact string helps when multiple cams exist)
inline constexpr std::string_view kCameraName{"Arducam OV9281 USB Camera 001"};
// inline constexpr std::string_view kCameraNameAlt{"Arducam OV9281 USB Camera 002"};

// Camera pose on robot: Transform from ROBOT frame -> CAMERA frame.
// Adjust these to match your real mounting.
//
// Example below (your current values):
//   - Camera is 0.50 m forward of robot center ( +X = 0.50 )
//   - Camera is centered left/right           (  Y = 0.00 )
//   - Camera is 0.50 m above robot center     ( +Z = 0.50 )
//   - Camera pitched DOWN by 30 degrees       (  pitch = -30° )
inline const frc::Transform3d kRobotToCam{
    frc::Translation3d{
        0.50_m,   // +X forward from robot center
        0.00_m,   // +Y left  (use negative for right)
        0.50_m    // +Z up
    },
    frc::Rotation3d{
        0_rad,     // roll
       -30_deg,    // pitch (DOWN is negative)
        0_rad      // yaw   (LEFT is positive, RIGHT is negative)
    }
};

// 2026 field tag layout (keep as selected for the current game)
inline const frc::AprilTagFieldLayout kTagLayout{
    frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2026RebuiltWelded)};

// Tilt threshold at which we begin de-weighting vision (degrees)
inline constexpr double kTiltThresholdDegrees = 15.0;

// --- Optional: expose smooth tilt-scaling tunables here so VisionPoseEstimator can use them ---
inline constexpr double kTiltGainPerDeg  = 0.15;   // additional scale per degree beyond threshold
inline constexpr double kTiltMaxScale    = 100.0;  // cap to keep estimator numerically stable

}  // namespace Vision

}  // namespace constants