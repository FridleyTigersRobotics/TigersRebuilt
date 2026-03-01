
// WPILib BSD license in project root.

#include "subsystems/Intake.h"
#include <algorithm>
#include <cmath>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h> 

#include "subsystems/Elevator.h"   // NEW: needed for AngleStowSafedCmd
#include <functional>              // NEW: for std::function


Intake::Intake() {
  ConfigureMotors_();

  // Safe startup
  StopWheels();
  m_angle.Set(0.0);
}

void Intake::ConfigureMotors_() {
  // ---------------- Wheels (velocity control) ----------------
  rev::spark::SparkMaxConfig wcfg;
  wcfg.Inverted(constants::Intake::kWheelsInverted);
  wcfg.SmartCurrentLimit(constants::Intake::kWheelsCurrentLimitA);
  wcfg.VoltageCompensation(constants::Intake::kVoltageCompensation);
  wcfg.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);

  // Velocity loop gains (RPM domain) + FF
  wcfg.closedLoop.P(constants::Intake::kWheelsP)
                 .I(constants::Intake::kWheelsI)
                 .D(constants::Intake::kWheelsD);
  wcfg.closedLoop.feedForward.kS(constants::Intake::kWheelsFF_kS)
                               .kV(constants::Intake::kWheelsFF_kV);

  // Optional: convert velocity to mechanism RPM instead of motor RPM
  wcfg.encoder.VelocityConversionFactor(constants::Intake::kMechRPMperMotorRPM);

  // ---------------- Angle (position control) ----------------
  rev::spark::SparkMaxConfig acfg;
  acfg.Inverted(constants::Intake::kAngleInverted);
  acfg.SmartCurrentLimit(constants::Intake::kAngleCurrentLimitA);
  acfg.VoltageCompensation(constants::Intake::kVoltageCompensation);
  acfg.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
  acfg.ClosedLoopRampRate(0.2);

  // Optional: convert encoder position to mechanism rotations
  acfg.encoder.PositionConversionFactor(constants::Intake::kDegPerMotorRot);

  // Position loop gains + FF (usually FF=0 unless you add gravity/motion FF)
  acfg.closedLoop.P(constants::Intake::kAngleP)
                 .I(constants::Intake::kAngleI)
                 .D(constants::Intake::kAngleD);
  acfg.closedLoop.feedForward.kS(constants::Intake::kAngleFF_kS)
                               .kV(constants::Intake::kAngleFF_kV);

  // Apply configurations
  m_wheels.Configure(wcfg,
                     rev::ResetMode::kResetSafeParameters,
                     rev::PersistMode::kPersistParameters);
  m_wheels.ClearFaults();

  m_angle.Configure(acfg,
                    rev::ResetMode::kResetSafeParameters,
                    rev::PersistMode::kPersistParameters);
  m_angle.ClearFaults();
}

void Intake::SetWheelsSpeedRPM(double rpm) {
  // If you configured encoder conversion, this value becomes mechanism RPM.
  m_wheelsPID.SetSetpoint(rpm, rev::spark::SparkBase::ControlType::kVelocity);
}

void Intake::StopWheels() {
  m_wheelsPID.SetSetpoint(0.0, rev::spark::SparkBase::ControlType::kVelocity);
}

void Intake::SetAngleDeg(double deg) {
  if (!m_homed) {
    // If we're not homed, ensure homing is running and remember this request
    if (!m_homingActive) {
      StartHoming();
    }
    m_pendingAngleDeg = deg;   // queue to apply after homing completes
    return;
  }

  // Normal path once homed
  m_angleSetpointDeg = deg;
  ApplyAngleSetpoint_();
}

double Intake::GetAngleDeg() const {
  return m_angleEnc.GetPosition();
}


bool Intake::IsStowed() const {
  const double err = std::abs(GetAngleDeg() - constants::Intake::kStowDeg);
  return err <= constants::Intake::kAngleNearToleranceDeg;
}


void Intake::ForceZeroAngleHere() {
  m_angleEnc.SetPosition(0.0);
}

void Intake::StartHoming() {
  m_homed = false;
  m_homingActive = true;
  m_homeStart = frc::Timer::GetFPGATimestamp();
  m_angleSetpointDeg.reset();
}

void Intake::RunHoming_() {
  if (!m_homingActive) return;

  // Drive toward the physical stop
  m_angle.Set(constants::Intake::kHomeSpeed);

  // Success: switch pressed
  if (HomeSwitchPressed_()) {
    m_angle.Set(0.0);
    m_angleEnc.SetPosition(0.0);
    m_homed = true;
    m_homingActive = false;

    // If we had a pending angle request, apply it now
    if (m_pendingAngleDeg.has_value()) {
      m_angleSetpointDeg = *m_pendingAngleDeg;
      m_pendingAngleDeg.reset();
      ApplyAngleSetpoint_();
    }
    return;
  }

  // Timeout protection
  if (frc::Timer::GetFPGATimestamp() - m_homeStart > constants::Intake::kHomeTimeout) {
    m_angle.Set(0.0);
    m_homingActive = false;
    m_homed = false; // not homed
    // Keep m_pendingAngleDeg as-is so a later SetAngleDeg() can re-trigger
  }
}

void Intake::ApplyAngleSetpoint_() {
  if (!m_homed || !m_angleSetpointDeg.has_value()) return;
  const double tgtDeg = *m_angleSetpointDeg;
  m_anglePID.SetSetpoint(tgtDeg, rev::spark::SparkBase::ControlType::kPosition);
}

void Intake::PublishTelemetry_() {
  if (!m_publishTelemetry) return;

  IntakeNetTable->PutBoolean("Homed", m_homed);
  IntakeNetTable->PutBoolean("HomingActive", m_homingActive);
  IntakeNetTable->PutBoolean("HomeSwitch", HomeSwitchPressed_());
  IntakeNetTable->PutNumber("AngleDeg", m_angleEnc.GetPosition());
  IntakeNetTable->PutNumber("WheelsRPM", m_wheelsEnc.GetVelocity());
}

void Intake::Periodic() {
  if (m_homingActive) {
    RunHoming_();
  } else {
    // Reassert setpoint periodically (helps after brownouts)
    ApplyAngleSetpoint_();
  }
  PublishTelemetry_();
}

frc2::CommandPtr Intake::ToggleAngleCmd() {
  return frc2::cmd::RunOnce([this] {
    const double curDeg = GetAngleDeg();

    const double stow   = constants::Intake::kStowDeg;
    const double intake = constants::Intake::kIntakeDeg;

    const double dStow   = std::abs(curDeg - stow);
    const double dIntake = std::abs(curDeg - intake);

    // Determine which target is CLOSEST to current angle
    const bool stowIsClosest = (dStow <= dIntake);

    // If we are already "near" one of them, that one counts as the closest
    // (prevents tiny noise from flipping during the press)
    double targetDeg = 0.0;
    if (dStow <= constants::Intake::kAngleNearToleranceDeg && dIntake > constants::Intake::kAngleNearToleranceDeg) {
      // Near stow -> go to stow: stow
      targetDeg = stow;
    } else if (dIntake <= constants::Intake::kAngleNearToleranceDeg && dStow > constants::Intake::kAngleNearToleranceDeg) {
      // Near intake -> go to intake: intake
      targetDeg = intake;
    } else {
      // Not clearly near either (or near both) -> choose opposite of the closest
      targetDeg = stowIsClosest ? stow : intake;
    }

    // Safe: SetAngleDeg() only applies if homed; otherwise it will start homing and queue this setpoint
    SetAngleDeg(targetDeg);
  });
}

frc2::CommandPtr Intake::AngleStowCmd() {
  return frc2::cmd::RunOnce([this] {
    // Safe: SetAngleDeg() only applies if homed; otherwise it will start homing and queue this setpoint
    SetAngleDeg(constants::Intake::kStowDeg);
  });
}

// Refuse to stow if the elevator is above a safe height.
// We pass 'elevator' to read its height predicate.
frc2::CommandPtr Intake::AngleStowSafedCmd(const Elevator& elevator) {
  auto blockedIf = [&elevator] {
    return elevator.IsAbove(constants::Intake::kNoStowAboveHeight);
  };

  auto underlying = AngleStowCmd();
  return std::move(underlying)                 // <-- make it an rvalue
      .Unless(std::function<bool()>(blockedIf))
      .FinallyDo([this, blockedIf] {
        if (blockedIf()) {
          IntakeNetTable->PutString("Interlock", "Intake stow blocked: elevator raised");
        }
      })
      .WithName("IntakeAngleStowSafed");
}

     


frc2::CommandPtr Intake::AngleIntakeCmd() {
  return frc2::cmd::RunOnce([this] {
    // Safe: SetAngleDeg() only applies if homed; otherwise it will start homing and queue this setpoint
    SetAngleDeg(constants::Intake::kIntakeDeg);
  });
}

// Re-home only (no target angle after)
frc2::CommandPtr Intake::RehomeCmd() {
  return frc2::cmd::Sequence(
           // Start homing: resets flags and begins motion toward the stop.
           frc2::cmd::RunOnce([this] {
             // Clear any prior state and begin a fresh homing attempt
             StartHoming();
             // Optional: publish a marker so you can see who initiated homing
             IntakeNetTable->PutString("HomeStatus", "Rehome requested");
           }, {this}),

           // Wait until homed, but don't hang forever.
           frc2::cmd::WaitUntil([this] { return m_homed; })
             .WithTimeout(constants::Intake::kHomeTimeout)
         )
         .FinallyDo([this] {
           if (!m_homed) {
             // Ensure the motor is stopped and publish a timeout marker.
             m_angle.Set(0.0);
             IntakeNetTable->PutString("HomeStatus", "Rehome timeout");
             // Leave m_homed = false, so later SetAngleDeg() will trigger homing again.
           } else {
             IntakeNetTable->PutString("HomeStatus", "Rehome complete");
           }
         })
         .WithName("IntakeRehome");
}

// Map right trigger 0..1 to angle kStowDeg..kIntakeDeg.
// Runs only while scheduled; on release, command ends so others can take control. Restores back to intake position.
frc2::CommandPtr Intake::AngleFromTriggerSupplierWhileHeldCmd(std::function<double()> get) {
  return frc2::cmd::RunEnd(
      // RUN — map trigger position to angle each scheduler loop while held
      [this, get] {
        double raw = get(); // expected 0..1
        // Apply deadband: below deadband, treat as 0 so the arm stays at intake end
        double t = (std::abs(raw) < constants::Intake::kTriggerDeadband) ? 0.0 : raw;
        t = std::clamp(t, constants::Intake::kTriggerMin, constants::Intake::kTriggerMax);

        // Map: t=0 -> kIntakeDeg, t=1 -> kStowDeg (linear interpolation)
        double cmdDeg =
            constants::Intake::kIntakeDeg
            - t * (constants::Intake::kIntakeDeg - constants::Intake::kStowDeg);

        cmdDeg=std::clamp(cmdDeg,(constants::Intake::kStowDeg+27.0),(constants::Intake::kIntakeDeg)); //+27 to not crash intake into elevator

        SetAngleDeg(cmdDeg);  // respects homing path
      },
      // END — when the trigger is released, return to intake angle
      [this] {
        SetAngleDeg(constants::Intake::kIntakeDeg);
      }
  ).WithName("IntakeAngleFromTriggerSupplierWhileHeld");
}

void Intake::SetWheelsPercent(double duty) {
  m_wheels.Set(std::clamp(duty, -1.0, 1.0));  // open-loop duty; persists until changed
}

frc2::CommandPtr Intake::WheelsPercentCmd(double percent) {
  const double duty = std::clamp(percent, -1.0, 1.0);

  return frc2::cmd::RunEnd(
    // RUN — set percent output
    [this, duty] {
      m_wheels.Set(duty);
    },

    // END — stop wheels when command ends
    [this] { this -> m_wheels.Set(0.0); }

  ).WithName("IntakeWheelsPercentCmd");
}