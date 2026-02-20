// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include "Constants.h"

// Forward-declare your constants; include the real header in the .cpp.
namespace constants {
namespace Indexer {
  // Expected in your Constants.h:
  // inline constexpr int kLowerMotorCanID = 28;
  // inline constexpr int kUpperMotorCanId = 29;
  // inline constexpr double kRunPercent   = 0.90;  // 0..1
  // inline constexpr double kUpperLowerRatio = 1.00; // upper = lower * ratio
} // namespace Indexer
} // namespace constants

class Indexer : public frc2::SubsystemBase {
public:
  Indexer();

  /** Called periodically whenever the CommandScheduler runs. */
  void Periodic() override;

  // -------- Mechanism control (open-loop duty) --------
  void Run();                                    // forward at constants::Indexer::kRunPercent
  void Reverse();                                 // reverse at constants::Indexer::kRunPercent
  void SetPercent(double magnitude, bool forward);
  void Stop();

  // -------- Command helpers (return CommandPtr) --------
  // Start motors at magnitude (0..1) and direction; stop on end/interrupt.
  frc2::CommandPtr RunCmd(double magnitude, bool forward);

  // Use constants::Indexer::kRunPercent in forward direction (hold-to-run style).
  frc2::CommandPtr RunSetCmd();

  // Use constants::Indexer::kRunPercent in reverse direction (hold-to-reverse style).
  frc2::CommandPtr ReverseSetCmd();

  // One-shot stop (InstantCommand style).
  frc2::CommandPtr StopCmd();

  // Telemetry getters
  double GetLowerApplied() const { return m_lowerApplied; }
  double GetUpperApplied() const { return m_upperApplied; }

private:
  void ConfigureMotors_();
  static double Clamp01_(double x);

  // Motors
  rev::spark::SparkMax m_lower{
      constants::Indexer::kLowerMotorCanID, rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkMax m_upper{
      constants::Indexer::kUpperMotorCanId, rev::spark::SparkLowLevel::MotorType::kBrushless};

  double m_lowerApplied{0.0};
  double m_upperApplied{0.0};

  std::shared_ptr<nt::NetworkTable> IndexerNetTable =
      nt::NetworkTableInstance::GetDefault().GetTable("2227/Indexer");
};