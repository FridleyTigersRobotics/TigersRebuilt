
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "subsystems/VisionPoseEstimator.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include <frc/DriverStation.h>


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

  //Intake Rehome
  m_driverController.Start().OnTrue(m_intake.RehomeCmd());
  
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
  


static constexpr units::second_t kXStanceHoldTime{1.5};
pathplanner::NamedCommands::registerCommand(
  "Xstance",
  m_drivetrain
    .cmdSetXStance()
    // Stop the command the moment autonomous is no longer enabled
    .Until([] { return !frc::DriverStation::IsAutonomousEnabled(); })
    // Or stop after the time cap, whichever happens first
    .WithTimeout(kXStanceHoldTime)
    // Ensure it doesn't keep running while disabled (default is already false; explicit for clarity)
    .IgnoringDisable(false)
);


units::second_t kIntakeReverseTime{1.0};
pathplanner::NamedCommands::registerCommand(
  "IntakeReverseTimed",
  std::move(
    m_intake.WheelsPercentCmd(constants::Intake::kOpenLoopOuttake)
      .WithTimeout(kIntakeReverseTime)
      .WithName("IntakeReverseTimed")
  )
);


// Tunables for this named event
kIntakeReverseTime = units::second_t{1.0};  // duration to reverse
pathplanner::NamedCommands::registerCommand(
  "PrepBot",
  std::move(
    frc2::cmd::Sequence(
      // 1) Reverse wheels first; WheelsPercentCmd stops them when the timeout ends
      m_intake.WheelsPercentCmd(constants::Intake::kOpenLoopOuttake).WithTimeout(kIntakeReverseTime),
      // 2) Lower intake to kIntakeDeg using your existing command
      m_intake.AngleIntakeCmd(),
      //    Wait until the intake is near kIntakeDeg
      frc2::cmd::WaitUntil([&] {
        return std::abs(m_intake.GetAngleDeg() - constants::Intake::kIntakeDeg)
               <= constants::Intake::kAngleNearToleranceDeg;
      }).WithTimeout(units::second_t{3.0}),// proceed even if not there yet
      // 3) Raise elevator to max height with intake interlock
      m_elevator.SetHeightSafedCmd(constants::Elevator::kMaxHeight, m_intake)
    )
    .WithName("PrepBot")
    .IgnoringDisable(false)
  )
);


pathplanner::NamedCommands::registerCommand(
  "ElevatorLower",
  std::move(
    m_elevator
      .SetHeightCmd(constants::Elevator::kMinHeight)  // homes if needed; waits at setpoint
      .WithName("ElevatorLower")
      .IgnoringDisable(false)
  )
);


pathplanner::NamedCommands::registerCommand(
  "ElevatorRaise",
  std::move(
    m_elevator
      .SetHeightSafedCmd(constants::Elevator::kMaxHeight, m_intake) // homes if needed, waits at setpoint; blocks going up if intake is stowed
      .WithName("ElevatorRaise")
      .IgnoringDisable(false)
  )
);


// ---------------- Tunables ----------------
static constexpr units::second_t kAimTimeout{5.00};          // how long to try aiming
static constexpr units::second_t kSpinupTimeout{3.00};       // max wait to reach "at speed"
static constexpr double          kAtSpeedTolRpm = 200.0;     // "at speed" band (RPM)
static constexpr units::second_t kFeedTime{5.00};            // indexer feed duration

pathplanner::NamedCommands::registerCommand(
  "AutoShoot",
  std::move(
    frc2::cmd::Sequence(
      // ===== 1) AIM (alliance-aware) =====
      // Run cmdAimAtHub in parallel until the timeout fires.
      frc2::cmd::Deadline(
        frc2::cmd::Wait(kAimTimeout),  // <-- deadline
        m_drivetrain.cmdAimAtHub(
          m_driverController.GetHID(),      // left stick idle => no translation
          /*fieldRelative=*/true,
          constants::RobotConst::kSchedulerTiming,
          0.25 //large deadband to prevent stick bumps
        )
      ),

      // ===== 2) SPIN-UP & 3) FEED (gated) =====
      // Shooter CalcAndSetShotCmd runs continuously; the deadline branch
      // gates the indexer so it only feeds after at-speed OR timeout.
      frc2::cmd::Deadline(
        frc2::cmd::Sequence(
          frc2::cmd::WaitUntil([&]{ return m_shooter.AtSpeed(kAtSpeedTolRpm); })
            .WithTimeout(kSpinupTimeout),
          m_indexer.RunSetCmd().WithTimeout(kFeedTime)  // stops on end
        ),
        m_shooter.CalcAndSetShotCmd()                   // End(): stop + hood=0°
      ),

      // ===== 4) Redundant safety cleanup (idempotent) =====
      m_indexer.StopCmd(),
      m_shooter.StopCommand(),
      m_shooter.SetHoodDegCommand(0.0)
    )
    .WithName("AutoShoot")
    .IgnoringDisable(false)
    // Abort immediately when autonomous ends
    .Until([] { return !frc::DriverStation::IsAutonomousEnabled(); })
  )
);

// Start and leave running
pathplanner::NamedCommands::registerCommand(
  "IntakeWheelsStart",
  frc2::cmd::RunOnce([this] {
    m_intake.SetWheelsPercent(constants::Intake::kOpenLoopIntake);  // persists after command ends
  }).WithName("IntakeWheelsStart")
);

// Stop (set duty to 0)
pathplanner::NamedCommands::registerCommand(
  "IntakeWheelsStop",
  frc2::cmd::RunOnce([this] {
    m_intake.SetWheelsPercent(0.0);    // stop open-loop
  }).WithName("IntakeWheelsStop")
);


// Tunables for this named event
static constexpr double              kFlipTargetDeg   = constants::Intake::kStowDeg+27.0;   // <-- set your target angle here
static constexpr units::second_t     kPauseTime       = 0.50_s; // <-- how long to pause at target
static constexpr units::second_t     kMoveTimeout     = 2.00_s; // safety: give up waiting after 2s
static constexpr double              kAngleNearTolDeg = constants::Intake::kAngleNearToleranceDeg;
pathplanner::NamedCommands::registerCommand(
  "intakeflip",
  std::move(
    frc2::cmd::Sequence(
      // 1) Go to target angle (auto-homes first if needed)
      frc2::cmd::RunOnce([&] {
        m_intake.SetAngleDeg(kFlipTargetDeg);
      }),

      //    Wait until near target, but allow a timeout so we don't hang
      frc2::cmd::WaitUntil([&] {
        return std::abs(m_intake.GetAngleDeg() - kFlipTargetDeg) <= kAngleNearTolDeg;
      }).WithTimeout(kMoveTimeout),

      // 2) Pause at the target angle
      frc2::cmd::Wait(kPauseTime),

      // 3) Return to kIntakeDeg
      frc2::cmd::RunOnce([&] {
        m_intake.SetAngleDeg(constants::Intake::kIntakeDeg);
      }),

      //    Wait until near kIntakeDeg (with the same safety timeout)
      frc2::cmd::WaitUntil([&] {
        return std::abs(m_intake.GetAngleDeg() - constants::Intake::kIntakeDeg) <= kAngleNearTolDeg;
      }).WithTimeout(kMoveTimeout)
    )
    .WithName("intakeflip")
    .IgnoringDisable(false)
  )
);


  autoChooser = pathplanner::AutoBuilder::buildAutoChooser();
  frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);
}