
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.h"

#include <frc/Timer.h>
#include "subsystems/VisionPoseEstimator.h"

// (Optional) If you rely on unit literals in this file:
#include <units/angle.h>
#include <units/velocity.h>
using namespace units::literals;

Drivetrain::Drivetrain() {
  m_gyro.Reset();
}

// This method will be called once per scheduler run
void Drivetrain::Periodic() {}

void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot,
                       bool fieldRelative,
                       units::second_t period) {
  // Build chassis speeds; if field-relative, use the current estimated heading.
  const auto chassisSpeeds = fieldRelative
                                 ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                                       xSpeed, ySpeed, rot,
                                       m_poseEstimator.GetEstimatedPosition().Rotation())
                                 : frc::ChassisSpeeds{xSpeed, ySpeed, rot};

  // Apply discretization for better behavior at lower loop rates.
  auto states = m_kinematics.ToSwerveModuleStates(
      frc::ChassisSpeeds::Discretize(chassisSpeeds, period));

  // Desaturate if any wheel exceeds the max speed.
  m_kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

  // Unpack and command the modules.
  auto [fl, fr, bl, br] = states;
  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);
}

void Drivetrain::UpdateOdometry() {
  // Update estimator with gyro + module positions
  m_poseEstimator.Update(
      m_gyro.GetRotation2d(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_backLeft.GetPosition(), m_backRight.GetPosition()});

  // Pull vision estimate and uncertainties (user-defined VisionPoseEstimator)
  const frc::Pose2d visionPose =
      VisionPoseEstimator::GetEstimatedGlobalPose(m_poseEstimator.GetEstimatedPosition());

  const units::second_t visionTimestamp = VisionPoseEstimator::GetLastTimestamp();
  const Eigen::Matrix<double, 3, 1> stdDevs = VisionPoseEstimator::GetLastStdDevs();

  // Convert Eigen -> wpi::array<double,3>
  const wpi::array<double, 3> visionStdDevArray{stdDevs(0), stdDevs(1), stdDevs(2)};

  // Fuse vision into the pose estimator
  m_poseEstimator.AddVisionMeasurement(visionPose, visionTimestamp, visionStdDevArray);

  // Telemetry (optional)
  const auto visPose = VisionPoseEstimator::GetEstimatedGlobalPose(m_poseEstimator.GetEstimatedPosition());
  DrivetrainNetTable->PutNumber("Vision X", visPose.X().to<double>());
  DrivetrainNetTable->PutNumber("Vision Y", visPose.Y().to<double>());
  DrivetrainNetTable->PutNumber("Vision Heading", visPose.Rotation().Degrees().to<double>());
}
