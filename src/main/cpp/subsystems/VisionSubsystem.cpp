// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/VisionSubsystem.h"

VisionSubsystem::VisionSubsystem() : camera1("Arducam OV9281 USB Camera 001"), camera2("Arducam OV9281 USB Camera 002") {
    camera1.SetDriverMode(false); // Driver mode is an unfiltered / normal view of the camera to be used while driving the robot.
    camera2.SetDriverMode(false); // Driver mode is an unfiltered / normal view of the camera to be used while driving the robot.
}


// This method will be called once per scheduler run
void VisionSubsystem::Periodic() {}
