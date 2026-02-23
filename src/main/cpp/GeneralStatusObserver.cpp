// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "GeneralStatusObserver.h"
#include <filesystem>
#include <iostream>


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

void GeneralStatusObserver::UpdateHubStatus() {
  // --- 2026 constants (REBUILT) ---
  static const double kTeleopTotalSec = 140.0;

  struct Window { const char* name; double upper; double lower; bool bothActive; };
  static const Window kTeleopWindows[] = {
    {"TRANSITION", 140.0, 130.0, true},
    {"SHIFT1",     130.0, 105.0, false},
    {"SHIFT2",     105.0,  80.0, false},
    {"SHIFT3",      80.0,  55.0, false},
    {"SHIFT4",      55.0,  30.0, false},
    {"ENDGAME",     30.0,   0.0, true}
  };

  // Alliance + Game Data
  std::optional<frc::DriverStation::Alliance> ourAlliance = frc::DriverStation::GetAlliance();
  std::string gameData = frc::DriverStation::GetGameSpecificMessage();
  bool dataIsR = (gameData.size() > 0) && (gameData[0] == 'R' || gameData[0] == 'r');
  bool dataIsB = (gameData.size() > 0) && (gameData[0] == 'B' || gameData[0] == 'b');
  bool haveValidGameData = (dataIsR || dataIsB);

  bool haveAlliance = ourAlliance.has_value();
  bool weAreInactiveFirst =
      haveAlliance && haveValidGameData &&
      ((ourAlliance.value() == frc::DriverStation::Alliance::kRed  && dataIsR) ||
       (ourAlliance.value() == frc::DriverStation::Alliance::kBlue && dataIsB));

  // Outputs
  std::string currentPhase = "UNKNOWN";
  std::string nextPhase    = "UNKNOWN";
  bool ourHubActiveNow     = false;
  bool hubStateKnown       = false;
  double secondsToNext     = -1.0;

  if (frc::DriverStation::IsAutonomous()) {
    // AUTO: both hubs active
    currentPhase = "AUTO";
    ourHubActiveNow = true;
    hubStateKnown = true;

    // GetMatchTime() -> units::second_t; use .value() for double
    units::second_t tRem = frc::DriverStation::GetMatchTime();  // units type
    secondsToNext = tRem.value();                               // double seconds
    nextPhase = "TRANSITION";

  } else if (frc::DriverStation::IsTeleop()) {
    // TELEOP: compute from teleop time remaining
    units::second_t tRem = frc::DriverStation::GetMatchTime();
    double teleopRem = tRem.value();      // may be -1 in some open modes; clamp below handles negatives

    // Manual clamp (no <algorithm>/<cmath>)
    if (teleopRem < 0.0) teleopRem = 0.0;
    else if (teleopRem > kTeleopTotalSec) teleopRem = kTeleopTotalSec;

    // Locate current window
    const Window* win = nullptr;
    for (const auto& w : kTeleopWindows) {
      if (teleopRem <= w.upper && teleopRem > w.lower) {
        win = &w; break;
      }
    }
    if (!win) {
      if (teleopRem == kTeleopTotalSec) {
        win = &kTeleopWindows[0];
      } else if (teleopRem == 0.0) {
        win = &kTeleopWindows[sizeof(kTeleopWindows)/sizeof(kTeleopWindows[0]) - 1];
      }
    }

    if (win) {
      currentPhase = win->name;
      secondsToNext = teleopRem - win->lower;

      // Find next phase label
      for (size_t i = 0; i < sizeof(kTeleopWindows)/sizeof(kTeleopWindows[0]); ++i) {
        if (std::string(kTeleopWindows[i].name) == currentPhase) {
          if (i + 1 < sizeof(kTeleopWindows)/sizeof(kTeleopWindows[0])) {
            nextPhase = kTeleopWindows[i + 1].name;
          } else {
            nextPhase = "MATCH-END";
          }
          break;
        }
      }

      if (win->bothActive) {
        // TRANSITION/ENDGAME → both hubs active
        ourHubActiveNow = true;
        hubStateKnown = true;
      } else {
        // SHIFTs need Game Data + alliance
        if (haveValidGameData && haveAlliance) {
          bool isShift1 = (currentPhase == "SHIFT1");
          bool isShift2 = (currentPhase == "SHIFT2");
          bool isShift3 = (currentPhase == "SHIFT3");
          bool isShift4 = (currentPhase == "SHIFT4");

          // Alliance that is inactive first → ACTIVE in Shifts 2 & 4; otherwise ACTIVE in 1 & 3.
          if (weAreInactiveFirst) {
            ourHubActiveNow = (isShift2 || isShift4);
          } else {
            ourHubActiveNow = (isShift1 || isShift3);
          }
          hubStateKnown = true;
        } else {
          hubStateKnown = false;
          ourHubActiveNow = false;  // meaningless when unknown
        }
      }
    }

  } else if (frc::DriverStation::IsDisabled()) {
    currentPhase = "DISABLED";
    // leave defaults for others
  }

  // Publish
  ObserverNetTable->PutString("Hub/CurrentPhase", currentPhase);
  ObserverNetTable->PutString("Hub/NextPhase", nextPhase);
  ObserverNetTable->PutNumber("Hub/SecondsToNextPhase", secondsToNext);
  ObserverNetTable->PutBoolean("Hub/IsOurHubActiveNow", ourHubActiveNow);
  ObserverNetTable->PutBoolean("Hub/IsHubStateKnown", hubStateKnown);
  ObserverNetTable->PutString("Hub/GameDataChar",
                              haveValidGameData ? std::string(1, dataIsR ? 'R' : 'B') : std::string(""));
  ObserverNetTable->PutBoolean("Hub/WeAreInactiveFirst", weAreInactiveFirst);
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
  UpdateHubStatus();
}

void GeneralStatusObserver::DeleteAllLogs() {
        // Directories to clean: Hoot logs and Tuner X logs
        std::filesystem::path logDirs[] = {
            "/home/lvuser/logs",    // Hoot logs
            "/home/lvuser/tunelog"  // Tuner X logs
        };

        for (const auto& dir : logDirs) {
            if (std::filesystem::exists(dir) && std::filesystem::is_directory(dir)) {
                try {
                    size_t count = 0;
                    for (const auto& entry : std::filesystem::directory_iterator(dir)) {
                        std::filesystem::remove_all(entry);
                        ++count;
                    }
                    // Plain text console output
                    std::cout << "[INFO] Deleted " << count << " files in: " << dir << std::endl;
                    std::cout.flush();  // ensure output appears immediately
                } catch (const std::filesystem::filesystem_error& e) {
                    std::cout << "[ERROR] Could not delete logs in " << dir
                              << ": " << e.what() << std::endl;
                    std::cout.flush();
                }
            } else {
                std::cout << "[INFO] Log directory does not exist: " << dir << std::endl;
                std::cout.flush();
            }
        }
    }