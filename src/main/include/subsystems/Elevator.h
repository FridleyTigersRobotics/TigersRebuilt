#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/Commands.h>
#include <frc/DigitalInput.h>
#include <frc/Timer.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <units/length.h>
#include <units/time.h>
#include <rev/SparkMax.h>

// REV 2026+ API
#include <rev/SparkMax.h>  // rev::spark::SparkMax, SparkRelativeEncoder, SparkClosedLoopController

class Elevator : public frc2::SubsystemBase {
 public:
  Elevator();

  // ---- Public API ----
  void RequestHome();                           // non-blocking homing start
  bool IsHomed() const { return m_leftHomed && m_rightHomed; }

  void SetHeight(units::meter_t height);        // request height (meters; clamps to soft limits)
  units::meter_t GetHeight() const;             // avg of both sides (meters)
  bool AtSetpoint() const;                      // within tolerance (your constants)
  void Stop();                                  // open-loop stop

  // Commands
  frc2::CommandPtr HomeCmd();                            // homing only
  frc2::CommandPtr SetHeightCmd(units::meter_t target);  // auto-home then mov
  frc2::CommandPtr SpinDownTestCmd(double pct, units::second_t time);
  frc2::CommandPtr SpinUpTestCmd(double pct, units::second_t time);


  void Periodic() override;

 private:
  // ---- Utility ----
  static double MetersToRot(units::meter_t m);
  static units::meter_t RotToMeters(double rot);

  bool LeftBottomPressed() const;   // applies active-low assumption; invert if needed
  bool RightBottomPressed() const;

  void ApplyPositionReferences();   // send SetReference with sync correction
  void UpdateNetTable() const;

 private:
  // ---- Hardware ----
  rev::spark::SparkMax m_left;
  rev::spark::SparkMax m_right;

  rev::spark::SparkRelativeEncoder m_leftEnc;
  rev::spark::SparkRelativeEncoder m_rightEnc;

  rev::spark::SparkClosedLoopController m_leftPID;
  rev::spark::SparkClosedLoopController m_rightPID;

  frc::DigitalInput m_leftBottom;
  frc::DigitalInput m_rightBottom;

  // ---- State ----
  bool  m_leftHomed  = false;
  bool  m_rightHomed = false;
  bool  m_homing     = false;
  double m_targetRot = 0.0;     // desired screw position [rotations]
  bool   m_hasTarget = false;

  units::second_t m_homeStartTime = 0_s;

  std::shared_ptr<nt::NetworkTable> ElevatorNetTable =
      nt::NetworkTableInstance::GetDefault().GetTable("2227/Elevator");
};