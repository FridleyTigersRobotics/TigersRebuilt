
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

RobotContainer::RobotContainer() : m_drivetrain(), m_shooter(m_drivetrain), m_elevator(), m_intake() {
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

  // Default shooter to stop spinning using its own command
  m_shooter.SetDefaultCommand(
      m_shooter.StopCommand()
  );

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
        //m_driverController.GetHID().SetRumble(frc::GenericHID::RumbleType::kBothRumble, 1.0);
      }, {&m_drivetrain}).ToPtr());

  //run shooter, simple given speed to test
  m_driverController.A().WhileTrue(m_shooter.SetRPMCommand(5000.0));

  //calculate shot and spin up shooter
  m_driverController.RightBumper().WhileTrue(
    m_shooter.CalcAndSetShotCmd()
  );
  
  //aim robot at alliance Hub
  m_driverController.RightStick().WhileTrue(
      m_drivetrain.cmdAimAtHub(
        m_driverController.GetHID(),   // passes a const frc::XboxController& (non-copy)
        /*fieldRelative=*/true,
        constants::RobotConst::kSchedulerTiming)
    );

  //test elevator
  //m_buttons.Button(1).OnTrue(m_elevator.SpinDownTestCmd(0.2,units::time::second_t {1.0}));
  //m_buttons.Button(2).OnTrue(m_elevator.SpinUpTestCmd(0.2,units::time::second_t {1.0}));

  //run elevator
  m_buttons.Button(1).OnTrue(m_elevator.HomeCmd());
  m_buttons.Button(2).OnTrue(m_elevator.SetHeightCmd(units::meter_t{0.02}));
  m_buttons.Button(3).OnTrue(m_elevator.SetHeightCmd(units::meter_t{0.03}));

  // Wheels RPM presets
  m_buttons.Button(4).WhileTrue(
    frc2::cmd::StartEnd(
      [this]{ m_intake.SetWheelsSpeedRPM(constants::Intake::kIntakeRPM); },
      [this]{ m_intake.StopWheels(); },
      {&m_intake}
    )
  );
  m_buttons.Button(5).WhileTrue(
    frc2::cmd::StartEnd(
      [this]{ m_intake.SetWheelsSpeedRPM(constants::Intake::kOuttakeRPM); },
      [this]{ m_intake.StopWheels(); },
      {&m_intake}
    )
  );

  // Homing and a preset angle
  m_buttons.Button(6).OnTrue(frc2::cmd::RunOnce([this]{ m_intake.StartHoming(); }, {&m_intake}).IgnoringDisable(true));
  m_buttons.Button(7).OnTrue(frc2::cmd::RunOnce([this]{ m_intake.SetAngleDeg(constants::Intake::kIntakeDeg); }, {&m_intake}));
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