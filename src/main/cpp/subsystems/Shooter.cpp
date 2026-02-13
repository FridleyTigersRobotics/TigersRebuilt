// src/main/cpp/subsystems/Shooter.cpp
#include "subsystems/Shooter.h"

#include <rev/config/SparkMaxConfig.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>

using constants::Shooter::kInvertMotor;
using constants::Shooter::kOpenLoopRampSeconds;
using constants::Shooter::kPID_D;
using constants::Shooter::kPID_I;
using constants::Shooter::kPID_P;
using constants::Shooter::kSmartCurrentLimit;
using constants::Shooter::kSmartCurrentLimit550;
using constants::Shooter::kVoltageCompSaturation;
// Feedforward (recommended for NEO Vortex direct-drive)
using constants::Shooter::kFF_kS;
using constants::Shooter::kFF_kV;
// using constants::Shooter::kFF_kA;  // Uncomment if you decide to use kA
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
    rev::spark::SparkMaxConfig cfg;
    cfg.Inverted(kInvertMotor);
    cfg.SmartCurrentLimit(kSmartCurrentLimit);
    cfg.VoltageCompensation(kVoltageCompSaturation);
    cfg.OpenLoopRampRate(kOpenLoopRampSeconds);
    cfg.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);

    // Flywheel closed-loop PIDF on-device (velocity in RPM)
    cfg.closedLoop.P(kPID_P).I(kPID_I).D(kPID_D);
    cfg.closedLoop.feedForward.kS(kFF_kS).kV(kFF_kV);

    m_shooterMotor.Configure(cfg,
                             rev::ResetMode::kResetSafeParameters,
                             rev::PersistMode::kPersistParameters);
    m_shooterMotor.ClearFaults();
  }

  // ---------- Follower (mirror the leader on-device) ----------
  {
    rev::spark::SparkMaxConfig fcfg;
    fcfg.Follow(m_shooterMotor, /*invert=*/true);
    fcfg.SmartCurrentLimit(kSmartCurrentLimit);
    fcfg.VoltageCompensation(kVoltageCompSaturation);
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
    hcfg.VoltageCompensation(kVoltageCompSaturation);
    hcfg.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);

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
}

// ---- Commands (flywheel RPM retained) ----
frc2::CommandPtr Shooter::SetRPMCommand(double rpm) {
  return frc2::cmd::RunOnce([this, rpm] { this->SetTargetRPM(rpm); }, {this});
}

void Shooter::SetTargetRPM(double rpm) {
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

// ---- Periodic ----
void Shooter::Periodic() {
  // --- Shooter flywheel (Spark on-device velocity loop) ---
  if (m_targetRPM.has_value()) {
    m_cl.SetSetpoint(*m_targetRPM, rev::spark::SparkLowLevel::ControlType::kVelocity);
  } else {
    m_shooterMotor.Set(0.0);
  }

  // Telemetry
  m_lastMeasuredRPM   = m_encoder.GetVelocity();  // RPM
  m_lastAppliedOutput = m_shooterMotor.GetAppliedOutput();

  // --- Hood homing + Spark Position control ---
  auto enc = m_hoodMotor.GetEncoder();  // degrees due to conversion factor

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
        (void)enc.SetPosition(0.0);  // zero hood at home (degrees)

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
          const double rpm = {0.0}; // m_distToRpm(shotdist); calculation here
          this->SetTargetRPM(rpm);
        },
        // end: ensure shooter is safe when command ends or is interrupted
        [this] { this->Stop(); },
        {this}
    );
}
