// src/main/cpp/subsystems/Shooter.cpp
#include "subsystems/Shooter.h"

#include <rev/config/SparkMaxConfig.h>
#include <rev/config/SparkFlexConfig.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>

using constants::Shooter::kInvertMotor;
using constants::Shooter::kOpenLoopRampSeconds;
using constants::Shooter::kPID_D;
using constants::Shooter::kPID_I;
using constants::Shooter::kPID_P;
using constants::Shooter::kSmartCurrentLimit;
using constants::Shooter::kSmartCurrentLimit550;
using constants::Shooter::kVoltageCompensation;
// Feedforward (recommended for NEO Vortex direct-drive)
using constants::Shooter::kFF_kS;
using constants::Shooter::kFF_kV;
using constants::Shooter::kFF_kA;  // Uncomment if you decide to use kA
using constants::Shooter::kHoodDio;
using constants::Shooter::kHoodActiveLow;
using constants::Shooter::kHoodDegPerMotorRot;
using constants::Shooter::kHood_kP;
using constants::Shooter::kHood_kI;
using constants::Shooter::kHood_kD;
using constants::Shooter::kHoodOutMin;
using constants::Shooter::kHoodOutMax;
using constants::Shooter::kHoodHomeDuty;

Shooter::Shooter(Drivetrain& driveidentity) : m_drive(driveidentity) {
  // ---- ensure stopped on startup ----
  m_targetRPM.reset();             // no setpoint active
  m_shooterMotor.Set(0.0);         // force output = 0 now (before Configure)
  m_cl.SetIAccum(0.0);             // clear integral accumulator

  // ---------- Shooter flywheel base configuration ----------
  {
    rev::spark::SparkFlexConfig cfg;
    cfg.Inverted(kInvertMotor);
    cfg.SmartCurrentLimit(kSmartCurrentLimit);
    cfg.VoltageCompensation(kVoltageCompensation);
    cfg.OpenLoopRampRate(kOpenLoopRampSeconds);
    cfg.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);

    // Flywheel closed-loop PIDF on-device (velocity in RPM)
    cfg.closedLoop.P(kPID_P).I(kPID_I).D(kPID_D);
    cfg.closedLoop.OutputRange(-1.0,1.0).IZone(150.0); //anti-windup
    cfg.closedLoop.feedForward.kS(kFF_kS).kV(kFF_kV);
    cfg.closedLoop.feedForward.kA(kFF_kA);

    m_shooterMotor.Configure(cfg,
                             rev::ResetMode::kResetSafeParameters,
                             rev::PersistMode::kPersistParameters);
    m_shooterMotor.ClearFaults();
  }

  // ---------- Follower (mirror the leader on-device) ----------
  {
    rev::spark::SparkFlexConfig fcfg;
    fcfg.Follow(m_shooterMotor, /*invert=*/true);
    fcfg.SmartCurrentLimit(kSmartCurrentLimit);
    fcfg.VoltageCompensation(kVoltageCompensation);
    fcfg.OpenLoopRampRate(kOpenLoopRampSeconds);
    fcfg.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);

    m_shooterMotorFollower.Configure(fcfg,
                                     rev::ResetMode::kResetSafeParameters,
                                     rev::PersistMode::kPersistParameters);
    m_shooterMotorFollower.ClearFaults();
  }

  // ---------- Hood motor setup (RIO DIO homing + Spark Position control) ----------
  {
    rev::spark::SparkMaxConfig hcfg;
    hcfg.SmartCurrentLimit(kSmartCurrentLimit550);
    hcfg.VoltageCompensation(kVoltageCompensation);
    hcfg.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
    hcfg.Inverted(true);

    // 1) Report encoder in *degrees at hood* so Spark position setpoints use degrees
    hcfg.encoder.PositionConversionFactor(kHoodDegPerMotorRot);

    // 2) Closed-loop position on-device; use primary encoder as feedback
    hcfg.closedLoop
        .SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder)
        .P(kHood_kP)
        .I(kHood_kI)
        .D(kHood_kD)
        .OutputRange(kHoodOutMin, kHoodOutMax);

    m_hoodMotor.Configure(hcfg,
                          rev::ResetMode::kResetSafeParameters,
                          rev::PersistMode::kPersistParameters);
    m_hoodMotor.ClearFaults();
  }

  UpdateNetTable();

  
  // Subscribe to operator-entered fields (Elastic widgets write to these)
  // Paths will be "/Shooter/CmdHoodDeg" and "/Shooter/CmdRPM"
  m_cmdHoodDegSub = ShooterNetTable->GetDoubleTopic("CmdHoodDeg")
                        .Subscribe(0.0, nt::PubSubOptions{.periodic = 0.02});
  m_cmdRpmSub     = ShooterNetTable->GetDoubleTopic("CmdRPM")
                        .Subscribe(0.0, nt::PubSubOptions{.periodic = 0.02});

  // Seed defaults so Elastic shows the fields immediately
  ShooterNetTable->GetEntry("CmdHoodDeg").SetDouble(0.0);
  ShooterNetTable->GetEntry("CmdRPM").SetDouble(0.0);

}

// ---- Commands (flywheel RPM retained) ----
frc2::CommandPtr Shooter::SetRPMCommand(double rpm) {
  return frc2::cmd::RunOnce([this, rpm] { this->SetTargetRPM(rpm); }, {this});
}

void Shooter::SetTargetRPM(double rpm) {
  // Anti-windup: if this is a large step change, clear the integral
  // to avoid carrying I from the previous operating point.
  // Keep this band in sync with your closedLoop.IZone() (150 RPM).
  if (m_targetRPM && std::fabs(*m_targetRPM - rpm) > 150.0) {
    m_cl.SetIAccum(0.0);
  }

  m_targetRPM = rpm;
}

// Optional convenience: schedule a one-shot SetHoodDeg()
frc2::CommandPtr Shooter::SetHoodDegCommand(double deg) {
  return frc2::cmd::RunOnce([this, deg] {
    this->SetHoodDeg(deg);
  }, {this});
}

frc2::CommandPtr Shooter::StopCommand() {
  return frc2::cmd::RunOnce([this] { this->Stop(); }, {this});
}

void Shooter::Stop() {
  m_targetRPM.reset();
  m_cl.SetIAccum(0.0);
  m_shooterMotor.Set(0.0);
}

// ---- SetHoodDeg with Option B behavior ----
// If HOMED: apply immediately. If NOT homed: queue the request and kick homing.
void Shooter::SetHoodDeg(double deg) {
  if (gHoodState == HoodState::kHomed) {
    // Already homed: apply immediately
    gHoodLastCmdDeg = deg;
    ShooterNetTable->PutNumber("HoodTargetDeg", deg);

    // Push to Spark on-device position control now
    auto hoodCl = m_hoodMotor.GetClosedLoopController();
    hoodCl.SetSetpoint(gHoodLastCmdDeg,
                       rev::spark::SparkLowLevel::ControlType::kPosition);
  } else {
    // Not homed: queue the request and ensure homing will begin
    m_hoodPendingDeg = deg;  // requires std::optional<double> in Shooter.h

    if (gHoodState != HoodState::kHoming) {
      gHoodState = HoodState::kHoming;
    }

    // Publish pending for visibility; do NOT overwrite active target until homed
    ShooterNetTable->PutNumber("HoodTargetDeg", m_hoodPendingDeg.value());
  }
}

// ---- Helpers ----
bool Shooter::AtSpeed(double tolRpm) const {
  if (!m_targetRPM.has_value()) return false;
  return std::abs(*m_targetRPM - m_lastMeasuredRPM) <= tolRpm;
}

void Shooter::Periodic() {
  // ===== Shooter flywheel (Spark on-device velocity loop) =====
  // Setpoint slew-rate limiter to reduce current spikes / breaker trips.
  // We ramp the *commanded* setpoint toward m_targetRPM at a bounded rate.
  constexpr double kLoopDtSec = 0.02;           // scheduler period ~20 ms
  constexpr double kMaxRpmRatePerSec = 8000.0;  // tune 7000–9000 RPM/s typical
  const double maxStep = kMaxRpmRatePerSec * kLoopDtSec;

  // Keep a persistent ramped setpoint without adding members (static retains value between calls)
  static double s_rampedSetpointRPM = 0.0;

  if (m_targetRPM.has_value()) {
    const double target = *m_targetRPM;
    const double errToGo = target - s_rampedSetpointRPM;
    const double step = std::clamp(errToGo, -maxStep, maxStep);
    s_rampedSetpointRPM += step;

    // Drive using the ramped setpoint
    m_cl.SetSetpoint(s_rampedSetpointRPM, rev::spark::SparkLowLevel::ControlType::kVelocity);
  } else {
    m_shooterMotor.Set(0.0);
    s_rampedSetpointRPM = 0.0;  // reset ramp when idle
  }

  // Telemetry
  m_lastMeasuredRPM   = m_encoder.GetVelocity();         // RPM
  m_lastAppliedOutput = m_shooterMotor.GetAppliedOutput();

  // ----- Anti-windup guard rails -----
  // If we're far from setpoint (outside IZone) or the controller is saturated
  // and still pushing in the error direction, clear the integral accumulator.
  if (m_targetRPM.has_value()) {
    const double err = *m_targetRPM - m_lastMeasuredRPM;

    // Keep this band in sync with your closedLoop.IZone() in the config.
    constexpr double kIEnableBandRPM = 300.0;  // e.g., 300; use 150 if you tightened IZone
    const bool saturated = std::fabs(m_lastAppliedOutput) >= 0.98;
    const bool sameSign  = (err * m_lastAppliedOutput) > 0.0; // pushing same direction as error

    if (std::fabs(err) > kIEnableBandRPM || (saturated && sameSign)) {
      m_cl.SetIAccum(0.0);  // reset integral on the device
    }
  }

  // ===== Hood homing + Spark Position control (unchanged) =====
  auto enc = m_hoodMotor.GetEncoder(); // degrees due to conversion factor
  switch (gHoodState) {
    case HoodState::kIdle:
      // If you want to delay homing, keep idle; default is kHoming at boot
      break;

    case HoodState::kHoming: {
      // Drive toward the limit switch until pressed
      m_hoodMotor.Set(kHoodHomeDuty);
      const bool pressed = kHoodActiveLow ? (!gHoodLimit.Get()) : gHoodLimit.Get();
      if (pressed) {
        // Stop and zero at home
        m_hoodMotor.Set(0.0);
        (void)enc.SetPosition(0.0); // zero hood at home (degrees)
        gHoodState = HoodState::kHomed;

        // If a pending setpoint was queued before homing, apply it now
        auto hoodCl = m_hoodMotor.GetClosedLoopController();
        if (m_hoodPendingDeg.has_value()) {
          gHoodLastCmdDeg = *m_hoodPendingDeg;
          ShooterNetTable->PutNumber("HoodTargetDeg", gHoodLastCmdDeg);
          hoodCl.SetSetpoint(gHoodLastCmdDeg,
                             rev::spark::SparkLowLevel::ControlType::kPosition);
          m_hoodPendingDeg.reset();
        } else {
          // No pending request; park at home angle (0 deg)
          hoodCl.SetSetpoint(0.0, rev::spark::SparkLowLevel::ControlType::kPosition);
        }
      }
      break;
    }

    case HoodState::kHomed: {
      // Normal hold; no need to re-send setpoint every loop unless desired.
      break;
    }
  }

  UpdateNetTable();
}

void Shooter::UpdateNetTable() {
  // Shooter velocity loop telemetry
  ShooterNetTable->PutBoolean("ClosedLoopEnabled", m_targetRPM.has_value());
  ShooterNetTable->PutNumber("TargetRPM", m_targetRPM.value_or(0.0));
  ShooterNetTable->PutNumber("MeasuredRPM", m_lastMeasuredRPM);
  ShooterNetTable->PutNumber("RPMError", m_targetRPM.has_value() ? (*m_targetRPM - m_lastMeasuredRPM) : 0.0);
  ShooterNetTable->PutBoolean("AtSpeed", AtSpeed(100.0));
  ShooterNetTable->PutNumber("AppliedOutput", m_lastAppliedOutput);
  ShooterNetTable->PutNumber("BusVoltage", m_shooterMotor.GetBusVoltage());
  ShooterNetTable->PutNumber("OutputCurrent", m_shooterMotor.GetOutputCurrent());
  ShooterNetTable->PutNumber("MotorTempC", m_shooterMotor.GetMotorTemperature());
  ShooterNetTable->PutNumber("MetersToHubGoal", MetersToHubGoal());

  // Faults/Warnings
  auto faults       = m_shooterMotor.GetFaults();
  auto stickyFaults = m_shooterMotor.GetStickyFaults();
  ShooterNetTable->PutNumber("FaultsRaw", faults.rawBits);
  ShooterNetTable->PutNumber("StickyFaultsRaw", stickyFaults.rawBits);
  ShooterNetTable->PutBoolean("Fault_Sensor", faults.sensor);
  ShooterNetTable->PutBoolean("Fault_Temp", faults.temperature);

  // Hood telemetry
  ShooterNetTable->PutBoolean("HoodHomed", gHoodState == HoodState::kHomed);
  ShooterNetTable->PutBoolean("HoodLS_Pressed", kHoodActiveLow ? (!gHoodLimit.Get()) : gHoodLimit.Get());
  ShooterNetTable->PutNumber("HoodTargetDeg", gHoodLastCmdDeg);
  ShooterNetTable->PutNumber("HoodPosDeg", m_hoodMotor.GetEncoder().GetPosition());

  // Optional pending telemetry for debugging
  ShooterNetTable->PutBoolean("HoodHasPending", m_hoodPendingDeg.has_value());
  if (m_hoodPendingDeg.has_value()) {
    ShooterNetTable->PutNumber("HoodPendingDeg", *m_hoodPendingDeg);
  }
}

units::meter_t Shooter::MetersToTarget (frc::Translation2d& targetXY){
  const frc::Pose2d robotPose = m_drive.getPose();
  const units::meter_t dist = robotPose.Translation().Distance(targetXY);
  return dist;  // meters
}

double Shooter::MetersToHubGoal (){
  frc::Translation2d allianceHubCoords;
  const auto alliance = frc::DriverStation::GetAlliance();
  if (alliance && alliance.value() == frc::DriverStation::Alliance::kRed) {
    allianceHubCoords = constants::Field::kRedHubCoord;
    } else {
      allianceHubCoords = constants::Field::kBlueHubCoord;
      }
  const units::meter_t hubdist = MetersToTarget(allianceHubCoords);
  return hubdist.value();  // meters
}

frc2::CommandPtr Shooter::CalcAndSetShotCmd() {
  return frc2::cmd::RunEnd(
        // run: updates each scheduler loop (~20ms by default)
        [this] {
          frc::Translation2d allianceHubCoords;
          const auto alliance = frc::DriverStation::GetAlliance();
          if (alliance && alliance.value() == frc::DriverStation::Alliance::kRed) {
            allianceHubCoords = constants::Field::kRedHubCoord;
          } else {
            allianceHubCoords = constants::Field::kBlueHubCoord;
          }
          const units::meter_t shotdist = MetersToTarget(allianceHubCoords);
          const Shooter::ShotParams shot = DistToShotParams(shotdist);
          this->SetHoodDeg(shot.deg);
          this->SetTargetRPM(shot.rpm);
        },
        // end: ensure shooter is safe when command ends or is interrupted
        [this] {  this->Stop();
                  this->SetHoodDeg(0.0);
                },
        {this}
    );
}

Shooter::ShotParams Shooter::DistToShotParams(units::meter_t shootdist) const {
  // === Limits ===
  constexpr double kMinAngleDeg = constants::Shooter::kMinAngleDeg;
  constexpr double kMaxAngleDeg = constants::Shooter::kMaxAngleDeg;
  constexpr double kMinRPM      = 2000.0;
  constexpr double kMaxRPM      = 5000.0;
  
  Shooter::ShotParams shot;
  double distance_m=shootdist.value();
  shot.deg = {0.00};
  shot.rpm = {0.00};
  
  double angle = 5.3597426826 * distance_m + 3.7063057550;
  if (angle < kMinAngleDeg) angle = kMinAngleDeg;
  if (angle > kMaxAngleDeg) angle = kMaxAngleDeg;

  // RPM: linear model + clamp
  double rpm = 20.7592471832 * distance_m + 3218.5162113353;

  // If you prefer steadier behavior, replace with a constant like:
  // double rpm = 3250.0;

  if (rpm < kMinRPM) rpm = kMinRPM;
  if (rpm > kMaxRPM) rpm = kMaxRPM;

  shot.deg = angle;
  shot.rpm = rpm;
  return shot;
}

frc2::CommandPtr Shooter::ApplyNtShotWhileHeldCmd() {
  return frc2::cmd::RunEnd(
    // run: follow Elastic every cycle (~20 ms by default)
    [this] {
      const double reqDeg = ClampHoodDeg(m_cmdHoodDegSub.Get());
      const double reqRpm = ClampRpm(m_cmdRpmSub.Get());
      this->SetHoodDeg(reqDeg);     // your SetHoodDeg handles homing/queueing
      this->SetTargetRPM(reqRpm);   // feeds your periodic velocity loop
    },
    // end: when the button is released or command is interrupted
    [this] {
      this->SetHoodDeg(0.0);  // park hood at 0°
      this->Stop();           // clears target, zero output
    },
    {this}
  );
}

double Shooter::ClampRpm(double rpmClamp) {
  constexpr double kMin = 0.0;
  constexpr double kMax = 6784.0; // free speed of neo vortex
  if (rpmClamp < kMin) return kMin;
  if (rpmClamp > kMax) return kMax;
  return rpmClamp;
}

double Shooter::ClampHoodDeg(double degClamp) {
  constexpr double kMinDeg = constants::Shooter::kMinAngleDeg;   // set to your mechanical min
  constexpr double kMaxDeg = constants::Shooter::kMaxAngleDeg;  // set to your mechanical max
  if (degClamp < kMinDeg) return kMinDeg;
  if (degClamp > kMaxDeg) return kMaxDeg;
  return degClamp;
}


frc2::CommandPtr Shooter::PassFarCmd() {
  return frc2::cmd::RunEnd(
    [this] {
      const double reqDeg = ClampHoodDeg(35.0);
      const double reqRpm = ClampRpm(4000.0);
      this->SetHoodDeg(reqDeg);     // your SetHoodDeg handles homing/queueing
      this->SetTargetRPM(reqRpm);   // feeds your periodic velocity loop
    },
    // end: when the button is released or command is interrupted
    [this] {
      this->SetHoodDeg(0.0);  // park hood at 0°
      this->Stop();           // clears target, zero output
    },
    {this}
  );
}


frc2::CommandPtr Shooter::PassShortCmd() {
  return frc2::cmd::RunEnd(
    [this] {
      const double reqDeg = ClampHoodDeg(35.0);
      const double reqRpm = ClampRpm(2000.0);
      this->SetHoodDeg(reqDeg);     // your SetHoodDeg handles homing/queueing
      this->SetTargetRPM(reqRpm);   // feeds your periodic velocity loop
    },
    // end: when the button is released or command is interrupted
    [this] {
      this->SetHoodDeg(0.0);  // park hood at 0°
      this->Stop();           // clears target, zero output
    },
    {this}
  );
}

frc2::CommandPtr Shooter::PassMidCmd() {
  return frc2::cmd::RunEnd(
    [this] {
      const double reqDeg = ClampHoodDeg(35.0);
      const double reqRpm = ClampRpm(3000.0);
      this->SetHoodDeg(reqDeg);     // your SetHoodDeg handles homing/queueing
      this->SetTargetRPM(reqRpm);   // feeds your periodic velocity loop
    },
    // end: when the button is released or command is interrupted
    [this] {
      this->SetHoodDeg(0.0);  // park hood at 0°
      this->Stop();           // clears target, zero output
    },
    {this}
  );
}

frc2::CommandPtr Shooter::CalcAndSetStaticCmd() {
  return frc2::cmd::RunEnd(
        // run: updates each scheduler loop (~20ms by default)
        [this] {
          frc::Translation2d allianceHubCoords;
          const auto alliance = frc::DriverStation::GetAlliance();
          if (alliance && alliance.value() == frc::DriverStation::Alliance::kRed) {
            allianceHubCoords = constants::Field::kRedHubCoord;
          } else {
            allianceHubCoords = constants::Field::kBlueHubCoord;
          }
          const units::meter_t shotdist = constants::Shooter::kStaticShotDist;
          const Shooter::ShotParams shot = DistToShotParams(shotdist);
          this->SetHoodDeg(shot.deg);
          this->SetTargetRPM(shot.rpm);
        },
        // end: ensure shooter is safe when command ends or is interrupted
        [this] {  this->Stop();
                  this->SetHoodDeg(0.0);
                },
        {this}
    );
}
