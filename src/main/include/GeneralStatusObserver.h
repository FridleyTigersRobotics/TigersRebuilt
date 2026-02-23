// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <memory.h>
#include <string.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include <frc/PowerDistribution.h>
#include <frc/DriverStation.h>


class GeneralStatusObserver {
 public:
    GeneralStatusObserver();
    void UpdateNetTable();
    void UpdateHubStatus();
    void DeleteAllLogs();

 private:
    frc::PowerDistribution ZipZap{1, frc::PowerDistribution::ModuleType::kRev};
    std::string DetermineAlliance();
    std::shared_ptr<nt::NetworkTable> ObserverNetTable = nt::NetworkTableInstance::GetDefault().GetTable("2227/Status Observer");
};
