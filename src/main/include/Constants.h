// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

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

namespace constants{
namespace OperatorConstants {

inline constexpr int kDriverControllerPort = 0;

}  // namespace OperatorConstants

namespace Vision {
inline constexpr std::string_view kCameraName{"Arducam OV9281 USB Camera 001"};
inline constexpr std::string_view kCameraNameAlt{"Arducam OV9281 USB Camera 002"};
inline const frc::Transform3d kRobotToCam{
    frc::Translation3d{0.5_m, 0.0_m, 0.5_m},
    frc::Rotation3d{0_rad, -30_deg, 0_rad}};
inline const frc::AprilTagFieldLayout kTagLayout{
    frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2026RebuiltWelded)};

inline const Eigen::Matrix<double, 3, 1> kSingleTagStdDevs{4, 4, 8};
inline const Eigen::Matrix<double, 3, 1> kMultiTagStdDevs{0.5, 0.5, 1};
}  // namespace Vision
} // namespace Constants