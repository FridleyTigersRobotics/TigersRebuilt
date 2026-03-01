
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "subsystems/VisionPoseEstimator.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>


#include <frc2/command/ProxyCommand.h>
#include <frc2/command/Commands.h>       // for cmd::None()


#include <frc2/command/button/Trigger.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/InstantCommand.h>
#include <units/time.h>
using namespace units::literals;

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

RobotContainer::RobotContainer() : m_drivetrain(), m_shooter(m_drivetrain), m_elevator(), m_intake(), m_indexer() {
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
                                       constants::RobotConst::kSchedulerTiming);
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
    ); // also valid: m_driverController.B().WhileTrue(m_drivetrain.cmdSetXStance());


  // POV Up: Zero gyro (wrap InstantCommand with .ToPtr() to satisfy OnTrue()) 
  m_driverController.POV(0).OnTrue(
      frc2::InstantCommand([this]{
        m_drivetrain.ZeroGyro();
      }, {&m_drivetrain}).ToPtr());

  //aim robot at alliance Hub
  m_driverController.RightStick().WhileTrue(
      m_drivetrain.cmdAimAtHub(
        m_driverController.GetHID(),   // passes a const frc::XboxController& (non-copy)
        /*fieldRelative=*/true,
        constants::RobotConst::kSchedulerTiming)
    );

  

  // Elevator Low
  m_buttons.Button(1).OnTrue(m_elevator.SetHeightSafedCmd(constants::Elevator::kMinHeight, m_intake));

  //Elevator High
  m_buttons.Button(2).OnTrue(m_elevator.SetHeightSafedCmd(constants::Elevator::kClimbHeight, m_intake));

  //Elevator Climb
  m_buttons.Button(3).OnTrue(m_elevator.SetHeightSafedCmd(constants::Elevator::kMaxHeight, m_intake));

  //Intake Stow
  m_driverController.X().OnTrue(m_intake.AngleStowSafedCmd(m_elevator));

  //Intake Intake
  m_driverController.A().OnTrue(m_intake.AngleIntakeCmd());

  // RobotContainer.cpp

// Condition for "right trigger is held beyond deadband"
frc2::Trigger rtHeld{[this] {
  return m_driverController.GetRightTriggerAxis() > constants::Intake::kTriggerDeadband;
}};
// While true, run the mapping command; on release, it ends.
rtHeld.WhileTrue(
  m_intake.AngleFromTriggerSupplierWhileHeldCmd(
    [this]{ return m_driverController.GetRightTriggerAxis(); }
  )
);


  m_buttons.Button(4).WhileTrue(m_shooter.PassShortCmd());
  m_buttons.Button(5).OnTrue(m_shooter.PassMidCmd());
  m_buttons.Button(6).WhileTrue(m_shooter.PassFarCmd());

    //Intake In
  m_driverController.RightBumper().WhileTrue(m_intake.WheelsPercentCmd(constants::Intake::kOpenLoopIntake));

  //Intake Out
  m_driverController.LeftBumper().WhileTrue(m_intake.WheelsPercentCmd(constants::Intake::kOpenLoopOuttake));
  
  //Index Shoot
  m_buttons.Button(8).WhileTrue(m_indexer.RunSetCmd());

  //Index Rev
  m_buttons.Button(9).WhileTrue(m_indexer.ReverseSetCmd());

  //run shooter from nettable
  m_buttons.Button(7).WhileTrue(m_shooter.CalcAndSetShotCmd());

  m_buttons.Button(10).OnTrue(m_elevator.HomeCmd());

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  
  if (auto* selected = autoChooser.GetSelected()) {
    // Wrap the existing Command* in a ProxyCommand (non-owning), then convert to CommandPtr
    return frc2::ProxyCommand(selected).ToPtr();
  }
  return frc2::cmd::None();
  // Run your example auto in autonomous
  //return autos::ExampleAuto(&m_subsystem);
}

void RobotContainer::BuildPathPlannerAutoChooser(){
  
  pathplanner::NamedCommands::registerCommand("Xstance",std::move(frc2::cmd::Parallel(
    m_drivetrain.cmdSetXStance()
  )));

  autoChooser = pathplanner::AutoBuilder::buildAutoChooser();
  frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);
}