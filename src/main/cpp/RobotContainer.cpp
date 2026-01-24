// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "subsystems/VisionPoseEstimator.h"


#include <frc2/command/button/Trigger.h>
#include <frc/DriverStation.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  studica::AHRS m_navx { studica::AHRS::NavXComType::kMXP_SPI };
  VisionPoseEstimator::SetNavX(&m_navx);

  // Configure the button bindings
  ConfigureBindings();

  // start net table
  ContainerNetTable = ContainerNetInst.GetTable("2227/RobotContainer");
  ContainerNetInst.StartServer();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  frc2::Trigger([this] {
    return m_subsystem.ExampleCondition();
  }).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_subsystem);
}

std::string RobotContainer::DetermineAlliance()
{
  std::optional<frc::DriverStation::Alliance> alliance = frc::DriverStation::GetAlliance();
  std::optional<int> position = frc::DriverStation::GetLocation();
  std::string allianceColor = "Unknown"; // Default value
  if (alliance.has_value()) {
      switch (alliance.value()) {
        case frc::DriverStation::Alliance::kRed:
          allianceColor = "Red";
          break;
        case frc::DriverStation::Alliance::kBlue:
          allianceColor = "Blue";
          break;
      }
    }
  std::string positionStr;
  if (position) {
          // Use std::stringstream to convert the int to a string
          std::stringstream ss;
          ss << *position;  // Dereference the std::optional
          positionStr = ss.str();
      } else {
          positionStr = "Unknown";
      }

    std::string allianceDisplay = "Alliance: " + allianceColor + " --- Position: " + positionStr;
    return allianceDisplay;
}

void RobotContainer::UpdateNetTable(){
  double totalCurrent = ZipZap.GetTotalCurrent();
  double robotVoltage = ZipZap.GetVoltage();
  ContainerNetTable->PutNumber("Total Current", totalCurrent);
  ContainerNetTable->PutNumber("Robot Voltage", robotVoltage);


  double matchTimeSeconds = frc::DriverStation::GetMatchTime().value(); // Convert to double
  // Convert match time to mm:ss format
  int minutes = static_cast<int>(matchTimeSeconds) / 60;
  int seconds = static_cast<int>(matchTimeSeconds) % 60;
  std::string timeStr = fmt::format("{:02}:{:02}", minutes, seconds);
 
  ContainerNetTable->PutString("Match_Time_Str",timeStr);
  ContainerNetTable->PutNumber("Match_Time",frc::DriverStation::GetMatchTime().value());
  ContainerNetTable->PutString("Alliance",DetermineAlliance());
}