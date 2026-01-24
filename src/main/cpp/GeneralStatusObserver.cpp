// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "GeneralStatusObserver.h"


GeneralStatusObserver::GeneralStatusObserver(){}


std::string GeneralStatusObserver::DetermineAlliance()
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

void GeneralStatusObserver::UpdateNetTable(){
  double totalCurrent = ZipZap.GetTotalCurrent();
  double robotVoltage = ZipZap.GetVoltage();
  ObserverNetTable->PutNumber("Total Current", totalCurrent);
  ObserverNetTable->PutNumber("Robot Voltage", robotVoltage);


  double matchTimeSeconds = frc::DriverStation::GetMatchTime().value(); // Convert to double
  // Convert match time to mm:ss format
  int minutes = static_cast<int>(matchTimeSeconds) / 60;
  int seconds = static_cast<int>(matchTimeSeconds) % 60;
  std::string timeStr = fmt::format("{:02}:{:02}", minutes, seconds);
 
  ObserverNetTable->PutString("Match_Time_Str",timeStr);
  ObserverNetTable->PutNumber("Match_Time",frc::DriverStation::GetMatchTime().value());
  ObserverNetTable->PutString("Alliance",DetermineAlliance());
}