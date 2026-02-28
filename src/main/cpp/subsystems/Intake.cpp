
// WPILib BSD license in project root.

#include "subsystems/Intake.h"
#include <algorithm>
#include <cmath>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h> 

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
  double tgtRot = DegToRot(deg);
  m_angleSetpointRot = tgtRot;
  ApplyAngleSetpoint_();
}

double Intake::GetAngleDeg() const {
  return RotToDeg(m_angleEnc.GetPosition());
}

void Intake::ForceZeroAngleHere() {
  m_angleEnc.SetPosition(0.0);
}

void Intake::StartHoming() {
  m_homed = false;
  m_homingActive = true;
  m_homeStart = frc::Timer::GetFPGATimestamp();
  m_angleSetpointRot.reset();
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
      m_angleSetpointRot = DegToRot(*m_pendingAngleDeg);
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
  if (!m_homed || !m_angleSetpointRot.has_value()) return;

  // --- Rate limit toward the final target ---
  // Assumes robot main loop is ~20 ms. If you run a different loop, adjust periodSec.
  constexpr double kLoopPeriodSec = constants::RobotConst::kSchedulerTiming.value()/1000;
  const double maxStepDeg = constants::Intake::kAngleMaxDegPerSec * kLoopPeriodSec;
  const double maxStepRot = DegToRot(maxStepDeg);

  const double curRot = m_angleEnc.GetPosition();
  const double tgtRot = *m_angleSetpointRot;
  const double errorRot = tgtRot - curRot;

  // If within one step, just go to target
  double nextRot = tgtRot;
  if (std::abs(errorRot) > maxStepRot) {
    nextRot = curRot + std::copysign(maxStepRot, errorRot);
  }

  // Command the *limited* intermediate setpoint each loop
  m_anglePID.SetSetpoint(nextRot, rev::spark::SparkBase::ControlType::kPosition);
}

void Intake::PublishTelemetry_() {
  if (!m_publishTelemetry) return;

  IntakeNetTable->PutBoolean("Homed", m_homed);
  IntakeNetTable->PutBoolean("HomingActive", m_homingActive);
  IntakeNetTable->PutBoolean("HomeSwitch", HomeSwitchPressed_());
  IntakeNetTable->PutNumber("AngleDeg", GetAngleDeg());
  IntakeNetTable->PutNumber("AngleRot", m_angleEnc.GetPosition());
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

frc2::CommandPtr Intake::AngleIntakeCmd() {
  return frc2::cmd::RunOnce([this] {
    // Safe: SetAngleDeg() only applies if homed; otherwise it will start homing and queue this setpoint
    SetAngleDeg(constants::Intake::kIntakeDeg);
  });
}

// Map right trigger 0..1 to angle kStowDeg..kIntakeDeg.
// Runs only while scheduled; on release, command ends so others can take control.
frc2::CommandPtr Intake::AngleFromTriggerSupplierWhileHeldCmd(std::function<double()> get) {
  return frc2::cmd::Run([this, get] {
    double raw = get();  // expected 0..1
    double t = (std::abs(raw) < constants::Intake::kTriggerDeadband) ? 0.0 : raw;
    t = std::clamp(t, constants::Intake::kTriggerMin, constants::Intake::kTriggerMax);
    const double cmdDeg = constants::Intake::kIntakeDeg - t * (constants::Intake::kIntakeDeg - constants::Intake::kStowDeg);
    // Uses your existing safety/homing path
    SetAngleDeg(cmdDeg);
  }).WithName("IntakeAngleFromTriggerSupplierWhileHeld");
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