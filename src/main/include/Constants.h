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
inline constexpr std::string_view kCameraName{"Arducam_OV9281_USB_Camera_001"};
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
        0.075_m,   // +X forward from robot center
        0.280_m,   // +Y left  (use negative for right)
        0.080_m    // +Z up
    },
    frc::Rotation3d{
        0_rad,     // roll
        30_deg,    // pitch (DOWN is negative)
        0_rad      // yaw   (LEFT is positive, RIGHT is negative), 0_rad, units::radian_t{std::numbers::pi}
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

inline constexpr units::millisecond_t msStaleCam = units::millisecond_t{250};

inline constexpr double kSnapDistanceMeters = 1.0;   // snap if odometry and vision differ by >1.0 m
inline constexpr double kSnapAngleDeg       = 20.0;  // snap if heading differs by >20 deg

}  // namespace Vision

namespace Shooter {
  // motor Can ID
  inline constexpr int kShooterCanId = 21;
  inline constexpr int kShooterFolllowerCanId = 22;

  // Electrical & safety (unchanged if you already set these)
  inline constexpr bool   kInvertMotor            = false;
  inline constexpr int    kSmartCurrentLimit      = 40;     // A
  inline constexpr double kVoltageCompSaturation  = 12.0;   // V
  inline constexpr double kOpenLoopRampSeconds    = 0.20;   // s

  // ---- Closed-loop (Spark MAX velocity) ----
  // PID (duty-cycle based loop on controller; units scale with your velocity units)
  inline constexpr double kPID_P  = 0.00010;   // Vortex + 3x 4" Colsons, 1:1
  inline constexpr double kPID_I  = 0.0;       // add later only if needed
  inline constexpr double kPID_D  = 0.00010;   // small damping to reduce overshoot

  // Feedforward (Spark expects these FF units by default)
  // kV ~= Volts per RPM. Ideal ≈ 12/6784 = 0.00177 V/RPM; add headroom for losses.
  inline constexpr double kFF_kV  = 0.00195;   // good starting slope for real-world losses
  inline constexpr double kFF_kS  = 0.20;      // static friction (~0.15–0.3 V typical)

  // Optional: if you plan to use acceleration feedforward
  inline constexpr double kFF_kA  = 0.00010;   // small; tune only if you observe accel error
}  // namespace Shooter

namespace Field {
    inline constexpr units::meter_t kBlueHubX{4.6256};
    inline constexpr units::meter_t kBlueHubY{4.0346};
    inline constexpr units::meter_t kRedHubX {11.9154};
    inline constexpr units::meter_t kRedHubY {4.0346};
    inline constexpr frc::Translation2d kRedHubCoord{kRedHubX, kRedHubY};
    inline constexpr frc::Translation2d kBlueHubCoord{kBlueHubX, kBlueHubY};
}  // namespace Field

namespace Driver {
    inline constexpr double kDefaultDeadband = 0.05;
    inline constexpr double kAimkP = 12.0; //6.0
    inline constexpr double kAimkI = 0.8; //0.8
    inline constexpr double kAimkD = 0.06; //0.06
    inline constexpr units::radian_t kAimTol = units::radian_t{2_deg}; // aiming tolerance ~2 degrees
    inline constexpr double kAutokPt = 5.0; // translation
    inline constexpr double kAutokIt = 0.0;
    inline constexpr double kAutokDt = 0.0;
    inline constexpr double kAutokPr = 5.0; // rotation
    inline constexpr double kAutokIr = 0.0;
    inline constexpr double kAutokDr = 0.0;
} // namespace Driver

namespace RobotConst {
    inline constexpr units::millisecond_t kSchedulerTiming = units::millisecond_t{20}; //20_ms
}// namespace RobotConst

}  // namespace constants