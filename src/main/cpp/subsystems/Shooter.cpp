// src/main/cpp/subsystems/Shooter.cpp
#include "subsystems/Shooter.h"

#include <rev/config/SparkMaxConfig.h>

using constants::Shooter::kInvertMotor;
using constants::Shooter::kOpenLoopRampSeconds;
using constants::Shooter::kPID_D;
using constants::Shooter::kPID_I;
using constants::Shooter::kPID_P;
using constants::Shooter::kSmartCurrentLimit;
using constants::Shooter::kVoltageCompSaturation;
// Feedforward (recommended for NEO Vortex direct-drive)
using constants::Shooter::kFF_kS;
using constants::Shooter::kFF_kV;
// using constants::Shooter::kFF_kA;  // Uncomment if you decide to use kA

Shooter::Shooter() {
  // ---- Safety first: ensure stopped on startup ----
  m_targetRPM.reset();             // no setpoint active
  m_shooterMotor.Set(0.0);         // force output = 0 now (before Configure)
  m_cl.SetIAccum(0.0);             // clear integral accumulator

  // ---------- Base motor configuration ----------
  rev::spark::SparkMaxConfig cfg;

  cfg.Inverted(kInvertMotor);
  cfg.SmartCurrentLimit(kSmartCurrentLimit);
  cfg.VoltageCompensation(kVoltageCompSaturation);
  cfg.OpenLoopRampRate(kOpenLoopRampSeconds);
  // Flywheels typically prefer coast so they don't abruptly grab game pieces
  cfg.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);

  // ---------- Closed-loop (PIDF) configuration ----------
  // PID gains (controller runs on the Spark at 1 ms loop)
  cfg.closedLoop
      .P(kPID_P)
      .I(kPID_I)
      .D(kPID_D);

  // Feedforward terms:
  // Spark expects kS in volts and kV in volts-per-RPM by default
  cfg.closedLoop.feedForward
      .kS(kFF_kS)
      .kV(kFF_kV);
  // Optional acceleration FF if you characterize it:
  // cfg.closedLoop.feedForward.kA(kFF_kA);

  // Apply and persist using the non-deprecated signature
  m_shooterMotor.Configure(
      cfg,
      rev::ResetMode::kResetSafeParameters,
      rev::PersistMode::kPersistParameters);

  // Optional: clear sticky faults at boot
  m_shooterMotor.ClearFaults();

  UpdateNetTable();
}

// ---- Commands ----
frc2::CommandPtr Shooter::SetRPMCommand(double rpm) {
  return frc2::cmd::RunOnce([this, rpm] { this->SetTargetRPM(rpm); }, {this});
}

void Shooter::SetTargetRPM(double rpm) {
  m_targetRPM = rpm;
}

frc2::CommandPtr Shooter::StopCommand() {
  return frc2::cmd::RunOnce([this] { this->Stop(); }, {this});
}

void Shooter::Stop() {
  m_targetRPM.reset();
  m_cl.SetIAccum(0.0);
  m_shooterMotor.Set(0.0);
}

// ---- Helpers ----
bool Shooter::AtSpeed(double tolRpm) const {
  if (!m_targetRPM.has_value()) return false;
  return std::abs(*m_targetRPM - m_lastMeasuredRPM) <= tolRpm;
}

// ---- Periodic ----
void Shooter::Periodic() {
  if (m_targetRPM.has_value()) {
    // Drive velocity loop (RPM) using the Spark's closed-loop controller
    m_cl.SetSetpoint(
        *m_targetRPM,
        rev::spark::SparkLowLevel::ControlType::kVelocity);
  } else {
    // No target => guarantee motor is commanded to 0 each loop
    m_shooterMotor.Set(0.0);
  }

  // Telemetry
  m_lastMeasuredRPM   = m_encoder.GetVelocity();      // RPM
  m_lastAppliedOutput = m_shooterMotor.GetAppliedOutput();

  UpdateNetTable();
}

void Shooter::UpdateNetTable() {
  frc::SmartDashboard::PutBoolean("Shooter/ClosedLoopEnabled", m_targetRPM.has_value());
  frc::SmartDashboard::PutNumber("Shooter/TargetRPM", m_targetRPM.value_or(0.0));
  frc::SmartDashboard::PutNumber("Shooter/MeasuredRPM", m_lastMeasuredRPM);
  frc::SmartDashboard::PutNumber(
      "Shooter/RPMError",
      m_targetRPM.has_value() ? (*m_targetRPM - m_lastMeasuredRPM) : 0.0);

  frc::SmartDashboard::PutNumber("Shooter/AppliedOutput", m_lastAppliedOutput);
  frc::SmartDashboard::PutNumber("Shooter/BusVoltage", m_shooterMotor.GetBusVoltage());
  frc::SmartDashboard::PutNumber("Shooter/OutputCurrent", m_shooterMotor.GetOutputCurrent());
  frc::SmartDashboard::PutNumber("Shooter/MotorTempC", m_shooterMotor.GetMotorTemperature());

  // Faults/Warnings (publish raw bitmasks + a couple booleans)
  auto faults       = m_shooterMotor.GetFaults();
  auto stickyFaults = m_shooterMotor.GetStickyFaults();
  frc::SmartDashboard::PutNumber("Shooter/FaultsRaw", faults.rawBits);
  frc::SmartDashboard::PutNumber("Shooter/StickyFaultsRaw", stickyFaults.rawBits);
  frc::SmartDashboard::PutBoolean("Shooter/Fault_Sensor", faults.sensor);
  frc::SmartDashboard::PutBoolean("Shooter/Fault_Temp", faults.temperature);
}