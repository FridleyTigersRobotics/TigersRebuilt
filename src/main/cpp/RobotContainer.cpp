
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "subsystems/VisionPoseEstimator.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include <frc2/command/button/Trigger.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/InstantCommand.h>
#include <units/time.h>
using namespace units::literals;

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

RobotContainer::RobotContainer() {
  // NOTE: Drivetrain owns navX now; do not set it here
  // VisionPoseEstimator::SetNavX(&m_navx);  // (removed)

  // Configure the button bindings
  ConfigureBindings();
  BuildPathPlannerAutoChooser();

  // Default teleop drive: read Xbox each loop and drive
  m_drivetrain.SetDefaultCommand(
      frc2::RunCommand(
          [this] {
            m_drivetrain.DriveFromXbox(m_driverController.GetHID(),
                                       /*fieldRelative=*/true,
                                       20_ms);
          },
          {&m_drivetrain}));
}

void RobotContainer::ConfigureBindings() {
  // Example trigger using your ExampleSubsystem
  frc2::Trigger([this] {
    return m_subsystem.ExampleCondition();
  }).OnTrue(ExampleCommand(&m_subsystem).ToPtr());


  // Hold B to X the wheels
    m_driverController.B().WhileTrue(
        frc2::RunCommand(
            [this] { m_drivetrain.SetXStance(); },   // runs every ~20 ms while held
            {&m_drivetrain}
        ).ToPtr()
    );


  // POV Up: Zero gyro (wrap InstantCommand with .ToPtr() to satisfy OnTrue()) 
  m_driverController.POV(0).OnTrue(
      frc2::InstantCommand([this]{
        m_drivetrain.ZeroGyro();
        m_driverController.GetHID().SetRumble(frc::GenericHID::RumbleType::kBothRumble, 1.0);
      }, {&m_drivetrain}).ToPtr());

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  auto factory = m_autoFactoryChooser.GetSelected();
  if (factory) {
    return factory();  // Build a fresh CommandPtr for the selected auto
  }
  // Fallback: no selection / no autos
  return frc2::InstantCommand([] {}).ToPtr();

  // Run your example auto in autonomous
  //return autos::ExampleAuto(&m_subsystem);
}

void RobotContainer::BuildPathPlannerAutoChooser(){
  // IMPORTANT: AutoBuilder must have been configured (typically in Drivetrain ctor)
  // BEFORE we build options. If not, chooser may be empty/flaky. [4](https://www.chiefdelphi.com/t/pathplanner-autobuilder-not-selecting-an-auto/454861)[5](https://pathplanner.dev/pplib-build-an-auto.html)

  // --- Build a chooser of factories (CommandPtr()) ---
  // Auto-discover all PathPlanner autos deployed with the project
  std::vector<std::string> autoNames = pathplanner::AutoBuilder::getAllAutoNames(); // [3](https://pathplanner.dev/api/cpp/classpathplanner_1_1AutoBuilder.html)

  if (!autoNames.empty()) {
    // Set a default option to the first auto
    const std::string defaultName = autoNames.front();
    m_autoFactoryChooser.SetDefaultOption(
        defaultName,
        [defaultName] {
          return pathplanner::AutoBuilder::buildAuto(defaultName); // returns CommandPtr [3](https://pathplanner.dev/api/cpp/classpathplanner_1_1AutoBuilder.html)
        });

    // Add the rest as selectable options
    for (size_t i = 1; i < autoNames.size(); ++i) {
      const std::string name = autoNames[i];
      m_autoFactoryChooser.AddOption(
          name,
          [name] {
            return pathplanner::AutoBuilder::buildAuto(name);      // CommandPtr
          });
    }
  } else {
    // No autos found; provide a safe no-op default
    m_autoFactoryChooser.SetDefaultOption(
        "Do Nothing",
        [] {
          // Any CommandPtr works here; InstantCommand().ToPtr() is simple and non-blocking
          return frc2::InstantCommand([] {}).ToPtr();
        });
  }

  // Publish the chooser
  frc::SmartDashboard::PutData("Auto Mode", &m_autoFactoryChooser);
}