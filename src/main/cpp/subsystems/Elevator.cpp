
#include "subsystems/Elevator.h"
#include "Constants.h"  // defines namespace constants::Elevator

#include <algorithm>
#include <cmath>
#include <units/math.h>              // units::math::abs for unit types
#include <rev/config/SparkMaxConfig.h>
#include <frc2/command/Commands.h>   // cmd::* factories


#include "subsystems/Intake.h"     // NEW: needed for SetHeightSafedCmd
#include <functional>              // NEW: for std::function


namespace c = constants::Elevator;

Elevator::Elevator()
  : m_left{c::kLeadLeftCanId,  rev::spark::SparkLowLevel::MotorType::kBrushless},
    m_right{c::kLeadRightCanId, rev::spark::SparkLowLevel::MotorType::kBrushless},
    m_leftEnc{m_left.GetEncoder()},
    m_rightEnc{m_right.GetEncoder()},
    m_leftPID{m_left.GetClosedLoopController()},
    m_rightPID{m_right.GetClosedLoopController()},
    m_leftBottom{c::kLeftBottomDio},
    m_rightBottom{c::kRightBottomDio} {
  // Configure both controllers via the 2026 config API
  rev::spark::SparkMaxConfig cfg;
  cfg.Inverted(c::kInvertMotor);
  cfg.SmartCurrentLimit(c::kSmartCurrentLimit);
  cfg.VoltageCompensation(c::kVoltageCompensation);
  cfg.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
  // PID gains/output range in slot 0
  cfg.closedLoop.P(c::kP).I(c::kI).D(c::kD);
  cfg.closedLoop.OutputRange(-1.0, 1.0);
  cfg.OpenLoopRampRate(c::kOpenLoopRampSeconds);
  cfg.ClosedLoopRampRate(c::kClosedLoopRampSeconds);

  m_left.Configure(cfg,  rev::ResetMode::kResetSafeParameters,  rev::PersistMode::kPersistParameters);
  m_right.Configure(cfg, rev::ResetMode::kResetSafeParameters,  rev::PersistMode::kPersistParameters);

  m_left.ClearFaults();
  m_right.ClearFaults();

  // Encoders zero at boot (final zero occurs at homing)
  m_leftEnc.SetPosition(0.0);
  m_rightEnc.SetPosition(0.0);

  // Start un-homed
  m_leftHomed  = false;
  m_rightHomed = false;
  m_homing     = false;
  m_hasTarget  = false;

  UpdateNetTable();
}

void Elevator::RequestHome() {
  m_homing = true;
  m_leftHomed = false;
  m_rightHomed = false;
  m_hasTarget = false;  // ignore pending targets while homing
  m_homeStartTime = frc::Timer::GetFPGATimestamp();
}

bool Elevator::LeftBottomPressed() const {
  return m_leftBottom.Get();
}

bool Elevator::RightBottomPressed() const {
  return m_rightBottom.Get();
}

double Elevator::MetersToRot(units::meter_t m) {
  return m.value() / c::kLeadMetersPerRev;  // rotations = meters / (m/rev)
}

units::meter_t Elevator::RotToMeters(double rot) {
  return units::meter_t{rot * c::kLeadMetersPerRev};
}

void Elevator::SetHeight(units::meter_t height) {
  // Clamp to software soft limits
  const auto clamped = std::clamp(height, c::kMinHeight, c::kMaxHeight);
  m_targetRot = MetersToRot(clamped);
  m_hasTarget = true;
}

units::meter_t Elevator::GetHeight() const {
  const double avgRot = 0.5 * (m_leftEnc.GetPosition() + m_rightEnc.GetPosition());
  return RotToMeters(avgRot);
}


bool Elevator::IsAbove(units::meter_t h) const {
  return GetHeight() > h;
}

bool Elevator::IsBelow(units::meter_t h) const {
  return GetHeight() < h; 
}


bool Elevator::AtSetpoint() const {
  // Must be homed and have a target
  if (!IsHomed() || !m_hasTarget) return false;
  const double errL = std::abs(m_targetRot - m_leftEnc.GetPosition());
  const double errR = std::abs(m_targetRot - m_rightEnc.GetPosition());
  const double errAvg = 0.5 * (errL + errR);
  return RotToMeters(errAvg) <= c::kTolerance;
}

void Elevator::Stop() {
  // Open-loop stop; closed-loop will resume when ApplyPositionReferences runs
  m_left.Set(0.0);
  m_right.Set(0.0);
}

void Elevator::ApplyPositionReferences() {
  // Must be homed and have a target
  if (!IsHomed() || !m_hasTarget) return;

  const double leftPos  = m_leftEnc.GetPosition();
  const double rightPos = m_rightEnc.GetPosition();

  // Cross-coupled sync correction
  const double skew = leftPos - rightPos;  // + => left higher
  double corrLeft  = std::clamp(-c::kSyncGainRotPerRot * skew, -c::kSyncMaxCorrRot,  c::kSyncMaxCorrRot);
  double corrRight = std::clamp(+c::kSyncGainRotPerRot * skew, -c::kSyncMaxCorrRot,  c::kSyncMaxCorrRot);

  // Skew deadband: ignore tiny skew to avoid chatter
  const auto skew_m = units::math::abs(RotToMeters(skew));
  if (skew_m < c::kSkewDeadband) {
    corrLeft = 0.0;
    corrRight = 0.0;
  }

  // Skew fault (units-aware abs)
  if (skew_m > c::kSkewFault) {
    Stop();
    ElevatorNetTable->PutBoolean("SkewFault", true);
    
    // Make the system "unhomed" so next SetHeightCmd() will home first
    m_leftHomed  = false;
    m_rightHomed = false;

    // Also clear any active target so we don't try to hold a setpoint while unhomed
    m_hasTarget = false;

    return;
  } else {
    ElevatorNetTable->PutBoolean("SkewFault", false);
  }

  // Issue position setpoints using 2026 API: SetSetpoint(..., ControlType::kPosition)
  m_leftPID.SetSetpoint(
      m_targetRot + corrLeft,
      rev::spark::SparkBase::ControlType::kPosition);
  m_rightPID.SetSetpoint(
      m_targetRot + corrRight,
      rev::spark::SparkBase::ControlType::kPosition);
}

frc2::CommandPtr Elevator::HomeCmd() {
  // Command owns the timeout; Periodic() should not do any timeout checks.
  return frc2::cmd::Sequence(
           // 1) Start homing and clear the UI timeout latch
           frc2::cmd::RunOnce([this] {
             ElevatorNetTable->PutBoolean("HomeTimeout", false);
             RequestHome();
           }, {this}),
           // 2) Wait until homed, but give up after the configured timeout
           frc2::cmd::WaitUntil([this] { return IsHomed(); })
             .WithTimeout(constants::Elevator::kHomeTimeout)
         )
         .FinallyDo([this] {
            if (!IsHomed()) {
              Stop();
              m_homing = false;
              ElevatorNetTable->PutBoolean("HomeTimeout", true);
            }
          })
         .WithName("ElevatorHome");
}

frc2::CommandPtr Elevator::SetHeightCmd(units::meter_t targetHeight) {
  // Build the branch at schedule time to avoid moving CommandPtr lvalues
  return frc2::cmd::Defer(
           [this, targetHeight] {
             auto IssueTarget = [this, targetHeight] {
               return frc2::cmd::RunOnce([this, targetHeight] { SetHeight(targetHeight); }, {this});
             };
             auto WaitAtSetpoint = [this] {
               return frc2::cmd::WaitUntil([this] { return AtSetpoint(); });
             };

             if (IsHomed()) {
               // Already homed: set target, then wait at setpoint
               return frc2::cmd::Sequence(IssueTarget(), WaitAtSetpoint());
             } else {
               // Not homed: Home first, then set target, then wait
               return frc2::cmd::Sequence(HomeCmd(), IssueTarget(), WaitAtSetpoint());
             }
           },
           {this}
         ).WithName("ElevatorSetHeight");
}

// Refuse to move to targetHeight if the intake is stowed.
// 'intake' is referenced so we can read its live state.
frc2::CommandPtr Elevator::SetHeightSafedCmd(units::meter_t targetHeight, const Intake& intake) {
  auto blockedIf = [this, &intake, targetHeight] {
    const bool goingUp = targetHeight > GetHeight();
    return goingUp && intake.IsStowed();
  };

  auto underlying = SetHeightCmd(targetHeight);
  return std::move(underlying)                 // <-- make it an rvalue
      .Unless(std::function<bool()>(blockedIf))
      .FinallyDo([this, blockedIf] {
        if (blockedIf()) {
          ElevatorNetTable->PutString("Interlock", "Elevator blocked: intake stowed");
        }
      })
      .WithName("ElevatorSetHeightSafed");
}


frc2::CommandPtr Elevator::SpinDownTestCmd(double pct, units::second_t time) {
  const double kSafePct = std::clamp(std::abs(pct), 0.0, 1.0);
  return frc2::cmd::StartEnd(
    [this, kSafePct] {
      // If negative is "down" on your robot, use -kSafePct; flip sign if needed.
      m_left.Set(-kSafePct);
      m_right.Set(-kSafePct);
    },
    [this] { Stop(); },
    {this}
  ).WithTimeout(time).WithName("ElevatorSpinDownTest");
}

frc2::CommandPtr Elevator::SpinUpTestCmd(double pct, units::second_t time) {
  const double kSafePct = std::clamp(std::abs(pct), 0.0, 1.0);
  return frc2::cmd::StartEnd(
    [this, kSafePct] {
      m_left.Set(+kSafePct);
      m_right.Set(+kSafePct);
    },
    [this] { Stop(); },
    {this}
  ).WithTimeout(time).WithName("ElevatorSpinUpTest");
}

void Elevator::Periodic() {
  // ----- HOMING (command owns timeout) -----
  if (m_homing) {
    // Left side: drive down until bottom switch PRESSED, then zero & mark homed
    if (!m_leftHomed) {
      if (LeftBottomPressed()) {
        m_left.Set(0.0);
        m_leftEnc.SetPosition(0.0);
        m_leftHomed = true;
      } else {
        // Ensure this moves toward the switch; if "down" is negative, keep it negative.
        m_left.Set(c::kHomeSpeedPct);
      }
    } else {
      m_left.Set(0.0);
    }

    // Right side: same as left
    if (!m_rightHomed) {
      if (RightBottomPressed()) {
        m_right.Set(0.0);
        m_rightEnc.SetPosition(0.0);
        m_rightHomed = true;
      } else {
        m_right.Set(c::kHomeSpeedPct);
      }
    } else {
      m_right.Set(0.0);
    }

    // Finish homing when both sides are homed
    if (m_leftHomed && m_rightHomed) {
      m_homing = false;         // allow command to observe IsHomed() and finish
      m_targetRot = 0.0;        // zero target at home
      m_hasTarget = true;
      ApplyPositionReferences(); // command may complete same loop; hold at zero
    }

    UpdateNetTable();
    return; // Skip normal control while homing
  }

  // ----- NORMAL CLOSED-LOOP -----
  ApplyPositionReferences();
  UpdateNetTable();
}

void Elevator::UpdateNetTable() const {
  ElevatorNetTable->PutBoolean("LeftHomed",  m_leftHomed);
  ElevatorNetTable->PutBoolean("RightHomed", m_rightHomed);
  ElevatorNetTable->PutBoolean("Homing",     m_homing);
  ElevatorNetTable->PutNumber ("LeftRot",     m_leftEnc.GetPosition());
  ElevatorNetTable->PutNumber ("RightRot",    m_rightEnc.GetPosition());
  ElevatorNetTable->PutNumber ("TargetRot",   m_targetRot);
  ElevatorNetTable->PutNumber ("Height_m",    GetHeight().value());

  // Publish live skew (meters) to help tune deadband and fault levels
  const double leftPos  = m_leftEnc.GetPosition();
  const double rightPos = m_rightEnc.GetPosition();
  const auto  skew_m    = RotToMeters(leftPos - rightPos);
  ElevatorNetTable->PutNumber ("Skew_m",      skew_m.value());

  ElevatorNetTable->PutBoolean("AtSetpoint",  AtSetpoint());
}
