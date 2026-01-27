
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/CommandGenericHID.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <functional>

#include "Constants.h"
#include "subsystems/ExampleSubsystem.h"
#include "subsystems/Drivetrain.h"
#include "Vision.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  // Driver controls
  frc2::CommandXboxController m_driverController{
      constants::OperatorConstants::kDriverControllerPort};

  // Subsystems
  ExampleSubsystem m_subsystem;
  Drivetrain m_drivetrain;

  // Vision: pass a no-op callback matching Vision's expected signature
  //   (frc::Pose2d, units::second_t, Eigen::Matrix<double,3,1>)
  Vision m_vision{
      [](frc::Pose2d, units::second_t, Eigen::Matrix<double, 3, 1>) {}};

  void ConfigureBindings();
  void BuildPathPlannerAutoChooser();

  frc::SendableChooser<std::function<frc2::CommandPtr()>> m_autoFactoryChooser;

  std::shared_ptr<nt::NetworkTable> ContainerNetTable =
      nt::NetworkTableInstance::GetDefault().GetTable("2227/RobotContainer");
};
