// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Indexer.h"
#include "Constants.h"

#include <algorithm>

Indexer::Indexer()
  : m_lower(constants::Indexer::kLowerMotorCanID, rev::spark::SparkLowLevel::MotorType::kBrushless),
    m_upper(constants::Indexer::kUpperMotorCanId,  rev::spark::SparkLowLevel::MotorType::kBrushless) {
  ConfigureMotors_();
  Stop();
}

void Indexer::Periodic() {
  // Optional telemetry
  IndexerNetTable->PutNumber("LowerDuty", m_lowerApplied);
  IndexerNetTable->PutNumber("UpperDuty", m_upperApplied);
  IndexerNetTable->PutNumber("Ratio", constants::Indexer::kUpperLowerRatio);
}

// ---------- Mechanism control ----------
void Indexer::ConfigureMotors_() {
  rev::spark::SparkMaxConfig lcfg;
  lcfg.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
  lcfg.SmartCurrentLimit(40);
  lcfg.SecondaryCurrentLimit(55, 0.200);

  rev::spark::SparkMaxConfig ucfg;
  ucfg.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
  ucfg.SmartCurrentLimit(40);
  ucfg.SecondaryCurrentLimit(55, 0.200);

  m_lower.Configure(lcfg,
                    rev::ResetMode::kResetSafeParameters,
                    rev::PersistMode::kPersistParameters);
  m_upper.Configure(ucfg,
                    rev::ResetMode::kResetSafeParameters,
                    rev::PersistMode::kPersistParameters);
}

double Indexer::Clamp01_(double x) {
  return std::clamp(x, 0.0, 1.0);
}

void Indexer::Run() {
  SetPercent(constants::Indexer::kRunPercent, /*forward=*/true);
}

void Indexer::Reverse() {
  SetPercent(constants::Indexer::kRunPercent, /*forward=*/false);
}

void Indexer::SetPercent(double magnitude, bool forward) {
  const double mag = Clamp01_(magnitude);
  const double sign = forward ? 1.0 : -1.0;

  const double lower_cmd = sign * mag;
  const double upper_cmd = std::clamp(lower_cmd * constants::Indexer::kUpperLowerRatio, -1.0, 1.0);

  m_lower.Set(lower_cmd);
  m_upper.Set(upper_cmd);

  m_lowerApplied = lower_cmd;
  m_upperApplied = upper_cmd;
}

void Indexer::Stop() {
  m_lower.Set(0.0);
  m_upper.Set(0.0);
  m_lowerApplied = 0.0;
  m_upperApplied = 0.0;
}

// ---------- Command helpers ----------
frc2::CommandPtr Indexer::RunCmd(double magnitude, bool forward) {
  // StartEnd ensures Stop() is called when the command ends or is interrupted.
  return frc2::cmd::StartEnd(
            // onStart
            [this, magnitude, forward] {
              this->SetPercent(magnitude, forward);
            },
            // onEnd
            [this] {
              this->Stop();
            },
            {this}  // requirement
         ).WithName(std::string("IndexerRunCmd(") +
                     (forward ? "FWD" : "REV") + ", " +
                     std::to_string(std::clamp(magnitude, 0.0, 1.0)) + ")");
}

frc2::CommandPtr Indexer::RunSetCmd() {
  return RunCmd(constants::Indexer::kRunPercent, /*forward=*/true).WithName("IndexerRunDefaultCmd");
}

frc2::CommandPtr Indexer::ReverseSetCmd() {
  return RunCmd(constants::Indexer::kRunPercent, /*forward=*/false).WithName("IndexerReverseDefaultCmd");
}

frc2::CommandPtr Indexer::StopCmd() {
  return frc2::cmd::RunOnce([this] { this->Stop(); }, {this}).WithName("IndexerStopCmd");
}