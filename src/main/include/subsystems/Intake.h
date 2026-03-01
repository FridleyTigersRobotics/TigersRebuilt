
#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>
#include <frc/Timer.h>

#include <rev/SparkMax.h>
#include <rev/SparkLowLevel.h>
#include <rev/config/SparkMaxConfig.h>
#include <rev/config/SparkBaseConfig.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/XboxController.h>

#include <units/angle.h>
#include <units/time.h>

#include <optional>
#include "Constants.h"

class Elevator;

class Intake : public frc2::SubsystemBase {
 public:
  Intake();

  // ---------- Wheels (velocity control) ----------
  void SetWheelsSpeedRPM(double rpm);
  void StopWheels();

  // ---------- Angle (position) ----------
  void   SetAngleDeg(double deg);         // ignored until homed
  double GetAngleDeg() const;             // telemetry
  bool   IsAngleHomed() const { return m_homed; }
  bool   IsStowed() const;

  // Homing to RIO DIO switch (non-blocking; handled in Periodic)
  void StartHoming();

  // Utility: zero encoder at current position (use carefully)
  void ForceZeroAngleHere();

  // Toggle telemetry to SmartDashboard
  void EnableTelemetry(bool enable) { m_publishTelemetry = enable; }

  frc2::CommandPtr ToggleAngleCmd();
  frc2::CommandPtr AngleStowCmd();
  frc2::CommandPtr AngleStowSafedCmd(const Elevator& elevator);
  frc2::CommandPtr AngleIntakeCmd();
  frc2::CommandPtr AngleFromTriggerSupplierWhileHeldCmd(std::function<double()> triggerSupplier);
  void SetWheelsPercent(double duty);
  frc2::CommandPtr RehomeCmd();
  
  // Run wheels at a fixed percent [-1, 1] while scheduled; stop on end.
  frc2::CommandPtr WheelsPercentCmd(double percent);



  void Periodic() override;

 private:
  // ----------- Hardware -----------
  rev::spark::SparkMax m_wheels{constants::Intake::kWheelsMotorCanId, rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkMax m_angle {constants::Intake::kAngleMotorCanId,  rev::spark::SparkLowLevel::MotorType::kBrushless};

  // Encoders
  rev::spark::SparkRelativeEncoder m_wheelsEnc = m_wheels.GetEncoder();
  rev::spark::SparkRelativeEncoder m_angleEnc  = m_angle.GetEncoder();

  // Controllers (for issuing setpoints only; all gains set via Configure())
  rev::spark::SparkClosedLoopController m_wheelsPID = m_wheels.GetClosedLoopController();
  rev::spark::SparkClosedLoopController m_anglePID  = m_angle.GetClosedLoopController();

  // RIO DIO limit switch (reverse end)
  frc::DigitalInput m_homeSwitch{constants::Intake::kHomeDIOChannel};

  // ----------- State -----------
  bool m_homed = false;
  bool m_homingActive = false;
  units::second_t m_homeStart{0_s};
  std::optional<double> m_angleSetpointDeg; // internal target in motor rotations
  bool m_publishTelemetry = true;

  // ----------- Internals -----------
  void ConfigureMotors_();  // sets inversion, current limit, idle, voltage comp, PID/FF, conversions
  void RunHoming_();
  void ApplyAngleSetpoint_();
  void PublishTelemetry_();

  // Switch helper: abstracts NO/NC wiring
  bool HomeSwitchPressed_() const {
    bool raw = m_homeSwitch.Get();              // true/false from RIO
    return constants::Intake::kHomeSwitchActiveLow ? !raw : raw;
  }

  std::shared_ptr<nt::NetworkTable> IntakeNetTable =
      nt::NetworkTableInstance::GetDefault().GetTable("2227/Intake");

  std::optional<double> m_pendingAngleDeg;
};
