// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.h"
#include <frc/Timer.h>
#include "subsystems/VisionPoseEstimator.h"

Drivetrain::Drivetrain()
{
    m_gyro.Reset();
    DrivetrainNetTable = DrivetrainNetInst.GetTable("2227 Drivetrain");
    DrivetrainNetInst.StartServer();
}

// This method will be called once per scheduler run
void Drivetrain::Periodic() {}

void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot, bool fieldRelative,
                       units::second_t period) {
  auto states =
      m_kinematics.ToSwerveModuleStates(frc::ChassisSpeeds::Discretize(
          fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                              xSpeed, ySpeed, rot,
                              m_poseEstimator.GetEstimatedPosition().Rotation())
                        : frc::ChassisSpeeds{xSpeed, ySpeed, rot},
          period));

  m_kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);
}

void Drivetrain::UpdateOdometry() {
  m_poseEstimator.Update(m_gyro.GetRotation2d(),
                         {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                          m_backLeft.GetPosition(), m_backRight.GetPosition()});

    
  frc::Pose2d visionPose =
      VisionPoseEstimator::GetEstimatedGlobalPose(
          m_poseEstimator.GetEstimatedPosition());

  units::second_t visionTimestamp =
      VisionPoseEstimator::GetLastTimestamp();

  Eigen::Matrix<double, 3, 1> stdDevs =
      VisionPoseEstimator::GetLastStdDevs();

  // Convert Eigen stdDevs to wpi::array<double, 3>
  wpi::array<double, 3> visionStdDevArray{stdDevs(0), stdDevs(1), stdDevs(2)};

  // Feed vision into the pose estimator
  m_poseEstimator.AddVisionMeasurement(visionPose, visionTimestamp, visionStdDevArray);

  //
  DrivetrainNetTable->PutNumber("Vision X", VisionPoseEstimator::GetEstimatedGlobalPose(m_poseEstimator.GetEstimatedPosition()).X().to<double>());
  DrivetrainNetTable->PutNumber("Vision Y", VisionPoseEstimator::GetEstimatedGlobalPose(m_poseEstimator.GetEstimatedPosition()).Y().to<double>());
  DrivetrainNetTable->PutNumber("Vision Heading", VisionPoseEstimator::GetEstimatedGlobalPose(m_poseEstimator.GetEstimatedPosition()).Rotation().Degrees().to<double>());

}