#pragma once

#include <optional>

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>     // frc2::cmd::RunOnce
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkMax.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/DigitalInput.h> 

#include "Constants.h"
#include "Drivetrain.h"

class Shooter : public frc2::SubsystemBase {
 public:
  Shooter(Drivetrain& driveidentity);

  // ---- Closed-loop RPM control ----
  frc2::CommandPtr SetRPMCommand(double rpm);
  void SetTargetRPM(double rpm);

  frc2::CommandPtr StopCommand();      // stops the controller & motor
  void Stop();

  frc2::CommandPtr SetHoodDegCommand(double deg);
  void SetHoodDeg(double deg);

  // shot calculation and set speed
  frc2::CommandPtr CalcAndSetShotCmd();

  // Helpers
  bool AtSpeed(double tolRpm = 100.0) const;
  units::meter_t MetersToTarget(frc::Translation2d& targetXY);

  void UpdateNetTable();
  void Periodic() override;

 private:
  Drivetrain& m_drive;
  // SparkMax + NEO (CAN)
  rev::spark::SparkMax m_shooterMotor{
      constants::Shooter::kShooterCanId,
      rev::spark::SparkLowLevel::MotorType::kBrushless};

  rev::spark::SparkMax m_shooterMotorFollower{
      constants::Shooter::kShooterFolllowerCanId,
      rev::spark::SparkLowLevel::MotorType::kBrushless};

  rev::spark::SparkMax m_hoodMotor{
      constants::Shooter::kHoodMotorCanId,
      rev::spark::SparkLowLevel::MotorType::kBrushless};

  // Handles (owned by Spark)
  rev::spark::SparkRelativeEncoder      m_encoder = m_shooterMotor.GetEncoder();
  rev::spark::SparkClosedLoopController m_cl      = m_shooterMotor.GetClosedLoopController();

  // Closed-loop target RPM (std::nullopt => stop)
  std::optional<double> m_targetRPM;
  double m_lastMeasuredRPM{0.0};
  double m_lastAppliedOutput{0.0};

  std::shared_ptr<nt::NetworkTable> ShooterNetTable =
      nt::NetworkTableInstance::GetDefault().GetTable("2227/Shooter");

  // Simple state for homing
  enum class HoodState { kIdle, kHoming, kHomed };

  frc::DigitalInput gHoodLimit{constants::Shooter::kHoodDio};
  HoodState         gHoodState      = HoodState::kIdle;  // start idle at boot
  double            gHoodLastCmdDeg = 0.0;                 // target degrees after homing
  
  // Pending hood target (degrees) that was requested while un-homed.
  // When the homing transition completes, Periodic() will apply this and clear it.
  std::optional<double> m_hoodPendingDeg;

  
  struct ShotParams {
    double rpm;  // shooter wheel speed
    double deg;  // hood angle
  };

  ShotParams DistToShotParams(units::meter_t shootdist) const;


};