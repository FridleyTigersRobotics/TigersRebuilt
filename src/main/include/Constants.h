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
inline constexpr int kButtonsPort = 1;
inline constexpr int kOtherPort = 2;
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
        0.050_m,   // +X forward from robot center
        0.000_m,   // +Y left  (use negative for right)
        0.419_m    // +Z up
    },
    frc::Rotation3d{
        0.0_rad,     // roll
        8.0_deg,    // pitch (DOWN is negative)
        0.0_rad      // yaw   (LEFT is positive, RIGHT is negative), 0_rad, units::radian_t{std::numbers::pi}
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
  inline constexpr int kShooterFollowerCanId = 22;
  inline constexpr int kHoodMotorCanId = 23;

  // Electrical & safety (unchanged if you already set these)
  inline constexpr bool   kInvertMotor            = false;
  inline constexpr int    kSmartCurrentLimit      = 60;     // A
  inline constexpr double kVoltageCompensation  = 12.0;   // V
  inline constexpr double kOpenLoopRampSeconds    = 0.05;   // s
  inline constexpr int    kSmartCurrentLimit550   = 10;

  // ---- Closed-loop (Spark MAX velocity) ----
  // PID (duty-cycle based loop on controller; units scale with your velocity units)
  inline constexpr double kPID_P  = 0.00007;   // Vortex + 3x 4" Colsons, 1:1
  inline constexpr double kPID_I  = 0.000001;       // add later only if needed
  inline constexpr double kPID_D  = 0.00012;   // small damping to reduce overshoot

  // Feedforward (Spark expects these FF units by default)
  // kV ~= Volts per RPM. Ideal ≈ 12/6784 = 0.00177 V/RPM; add headroom for losses.
  inline constexpr double kFF_kV  = 0.00180;   // good starting slope for real-world losses
  inline constexpr double kFF_kS  = 0.20;      // static friction (~0.15–0.3 V typical)

  // Optional: if you plan to use acceleration feedforward
  inline constexpr double kFF_kA  = 0.00000;   // small; tune only if you observe accel error

  //Hood Constants
  inline constexpr int    kHoodDio               = 0;       // DIO channel for hood homing switch
  inline constexpr bool   kHoodActiveLow         = false;    // true if pressed pulls DIO low
  inline constexpr double kHoodDegPerMotorRot    = 1.00;     // 360.0 / gearRatio_to_hood
  inline constexpr double kMinAngleDeg = 0.0;
  inline constexpr double kMaxAngleDeg = 40.0;
  
  // Spark on-device PID for hood position (units are degrees due to conversion factor)
  inline constexpr double kHood_kP               = 0.12;
  inline constexpr double kHood_kI               = 0.0;
  inline constexpr double kHood_kD               = 0.0;
  // Limit output of the Spark’s position loop
  inline constexpr double kHoodOutMin            = -1.0;
  inline constexpr double kHoodOutMax            =  1.0;
  // Homing duty toward the switch while disabled CL
  inline constexpr double kHoodHomeDuty          = -0.05;   // sign to drive toward switch
  inline constexpr units::second_t kHomingTimeout = units::second_t{5.0};
  inline constexpr units::meter_t kStaticShotDist = units::meter_t{2.5};


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

namespace Elevator {
    inline constexpr int kLeadLeftCanId = 24;
    inline constexpr int kLeadRightCanId = 25;
    inline constexpr int kLeftBottomDio  = 1;
    inline constexpr int kRightBottomDio = 2;
    inline constexpr double kOpenLoopRampSeconds    = 0.05;   // s
    inline constexpr double kClosedLoopRampSeconds    = 0.05;   // s

    // Geometry
    inline constexpr double kLeadPitch = 0.002; // TR12x2 lead (2 mm/rev)
    inline constexpr double kSprocketRatio = ( 12.0 / 20.0 ); // 12 tooth sproket on motor, 20 tooth sproket on shaft
    inline constexpr double kLeadMetersPerRev = ( kLeadPitch * kSprocketRatio ); 

    // Range & tolerances
    inline constexpr units::meter_t kMinHeight = 0.00_m;
    inline constexpr units::meter_t kMaxHeight = 0.113_m;
    inline constexpr units::meter_t kClimbHeight = 0.100_m;
    inline constexpr units::meter_t kTolerance = 0.003_m; // ±3 mm

    // REV PID
    inline constexpr double kP = 0.50;
    inline constexpr double kI = 0.0;
    inline constexpr double kD = 0.0;
    inline constexpr double kFF = 0.0;

    // Limits
    inline constexpr int kSmartCurrentLimit = 60; //increase for NEOs
    inline constexpr bool kInvertMotor = true;
    inline constexpr double kVoltageCompensation = 12.0;

    // Homing
    inline constexpr double kHomeSpeedPct = -0.20; // negative = down
    inline constexpr units::second_t kHomeTimeout = 8_s;

    // Sync / skew
    inline constexpr double kSyncGainRotPerRot = 0.10;
    inline constexpr double kSyncMaxCorrRot    = 0.25;

    // Fault when skew exceeds this
    inline constexpr units::meter_t kSkewFault    = 0.015_m;  // 15 mm

    // Deadband: ignore tiny skew so we don't chatter the cross-coupled correction.
    // Start at ~10–20% of kSkewFault (1.5–3.0 mm here) and tune on-robot.
    inline constexpr units::meter_t kSkewDeadband = 0.002_m;  // 2 mm to start

} // namespace Elevator

namespace RobotConst {
    inline constexpr units::millisecond_t kSchedulerTiming = units::millisecond_t{20}; //20_ms
    inline constexpr double kPi = std::numbers::pi_v<double>;
}// namespace RobotConst

namespace Intake {

    // ---------------------------------
    // CAN IDs & electrical
    // ---------------------------------
    inline constexpr int  kWheelsMotorCanId      = 26;
    inline constexpr int  kAngleMotorCanId       = 27;

    inline const bool     kWheelsInverted        = false;
    inline const bool     kAngleInverted         = false;

    inline const int      kWheelsCurrentLimitA   = 20;  // A
    inline const int      kAngleCurrentLimitA    = 30;  // A
    inline const double   kVoltageCompensation   = 12.0;

    
    // Wheels velocity loop (mechanism RPM domain)
    inline constexpr double kWheelsP       = 0.0008;   // start here
    inline constexpr double kWheelsI       = 0.0;
    inline constexpr double kWheelsD       = 0.0;
    inline constexpr double kWheelsFF_kS   = 0.0;      // add only if needed
    inline constexpr double kWheelsFF_kV   = 12.0 / 1733.0; // ≈ 0.0069 V/RPM
    inline constexpr double kIntakeRPM     = -1500.0;   // if you keep 2000, note the mech free ≈1733; consider 1500–1700 instead
    inline constexpr double kOuttakeRPM    = 1500.0;
    inline constexpr double kMechRPMperMotorRPM = 1.0 / 3.0;
    inline constexpr double kOpenLoopIntake = -1.0;
    inline constexpr double kOpenLoopOuttake = 1.0;

    // Angle position loop
    inline constexpr double kAngleP = 0.025;
    inline constexpr double kAngleI = 0.0;
    inline constexpr double kAngleD = 0.0;
    inline constexpr double kAngleFF_kS = 0.0;       // usually 0
    inline constexpr double kAngleFF_kV = 0.0;       // usually 0
    inline constexpr double kDegPerMotorRot = 360.0 / 113.0;  // 113:1 gear reduction

    inline constexpr double kIntakeSpeed = 5500.00;

    inline constexpr double kTriggerDeadband = 0.05;
    inline constexpr double kTriggerMin = 0.0;
    inline constexpr double kTriggerMax = 1.0;

    // Max angle slew rate (degrees per second). Tune to taste (e.g., 90.0 = 1/4 turn per second if 360°)
    inline constexpr double kAngleMaxDegPerSec = 45.0;

    inline constexpr units::meter_t kNoStowAboveHeight = 0.006_m; // <-- tune




    // Geometry conversion:
    // If 120° pivot travel corresponds to 24 motor rotations:
    //   rotations/deg = 24 / 120 = 0.20
    inline constexpr double kRotationsPerDegree = (1 / kDegPerMotorRot);  //2.69 before

    // Useful preset angles (degrees) — optional
    inline constexpr double kStowDeg       = 0.0;
    inline constexpr double kIntakeDeg     = 94.0; //96 before

    // ---------------------------------
    // Homing: RIO DIO limit switch
    //   Wire the switch to DIO and GND. If logic is inverted at runtime,
    //   flip kHomeSwitchActiveLow.
    // ---------------------------------
    inline constexpr int                kHomeDIOChannel      = 3;        // your configuration
    inline constexpr bool               kHomeSwitchActiveLow = false;    
    inline constexpr double             kHomeSpeed           = -0.25;    // duty cycle toward switch (flip sign if wrong direction)
    inline constexpr units::second_t    kHomeTimeout         = 5.0_s;    // max allowed homing time
    inline constexpr double kAngleNearToleranceDeg = 2.0;

} // namespace Intake

namespace Indexer {
    inline constexpr int kLowerMotorCanID = 28;
    inline constexpr int kUpperMotorCanId = 29;
    inline constexpr double kRunPercent = 1.00; //0.90
    inline constexpr double kUpperLowerRatio = 1.00;
    inline constexpr bool kLowerInverted = false;
    inline constexpr bool kUpperInverted = false;
} // namespace Indexer


}  // namespace constants