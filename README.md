# Missile Guidance System for Space Engineers

This repository contains a C# script designed for the game **Space Engineers**, implementing an advanced missile guidance and control system. The script enables missiles to navigate towards targets using proportional navigation guidance algorithms, manage thrusters and gyroscopes for control, and detonate warheads upon reaching the destination.

## Features

- **Proportional Navigation Guidance (PNG)**: Implements a proportional navigation algorithm to steer the missile towards moving targets effectively.
- **Dynamic Navigation Constant**: Adjusts the navigation constant (`_navConstant`) dynamically based on missile speed and distance to the target for improved responsiveness.
- **Lofted Trajectory Support**: Adds intermediate waypoints for lofted trajectories, allowing top-down attack profiles.
- **Gyroscope and Thruster Control**: Overrides gyroscopes and thrusters to control missile orientation and speed precisely.
- **Detonation Logic**: Determines optimal detonation timing using proximity checks and raycasting.
- **Bombing Simulation**: Simulates bomb drops to predict impact points considering gravity and drag.
- **User Interface**: Displays mission-critical information on an in-game LCD, including distance to target, time to impact, and current speed.
- **Target Acquisition**: Uses onboard radar (turret) to detect and update target positions dynamically.

## Code Overview

### Main Components

- **Program Class**: Inherits from `MyGridProgram`, serving as the main entry point for the script.
- **Initialization**: Sets up references to essential blocks like thrusters, gyros, remote control, warheads, sensors, and programmable blocks.
- **Main Logic**: Handles the missile's operational flow, including start-up checks, guidance updates, control mechanisms, and detonation conditions.

### Guidance System

- **GuidanceBase Class**: Abstract base class providing the framework for guidance algorithms.
- **RelNavGuidance Class**: Abstract class extending `GuidanceBase` for relative navigation strategies.
- **ProNavGuidance Class**: Implements proportional navigation, calculating lateral acceleration for target interception.

### Key Methods

- `SimulateBombDrop(...)`: Predicts bomb trajectories accounting for gravity, drag, and steering limitations.
- `EvaluateBombingSuccess()`: Assesses the likelihood of a successful bombing run based on simulated impact.
- `CalculateMissileAcceleration()`: Computes potential acceleration using thrust and mass.
- `ComputeRotationVector(...)`: Determines the rotation vector needed to align with the desired acceleration.
- `ApplyGyroOverride(...)`: Applies rotation inputs to gyroscopes for missile steering.
- `AddLoftedTrajectoryWaypoints(...)`: Adds waypoints to create a lofted trajectory for top-down attacks.
- `DetonateWarheads()`: Triggers warhead detonation under specified conditions.

### User Interface

- **LCD Display**: Provides real-time mission data, including velocity, time to impact, and distance to target.
- **Visual Effects**: Includes animated progress bars and scrolling messages for enhanced feedback.

## Usage

1. **Setup**:
   - Place the script in a programmable block named appropriately (e.g., "JETOS Programmable Block").
   - Ensure all required blocks are present and correctly named (e.g., "Remote Control Missile", "Holo LCD", "Sensor").
2. **Configuration**:
   - Set up the programmable block's custom data with necessary parameters (e.g., target GPS coordinates, mode flags like `Topdown` or `AntiAir`).
3. **Initialization**:
   - The missile initializes and identifies its bay number based on the closest merge block.
4. **Launch Sequence**:
   - The missile waits for target information before starting.
   - Once a target is designated, the missile begins guidance and control operations.
5. **In-Flight Operation**:
   - The missile uses the `ProNavGuidance` system to navigate toward the target.
   - It adjusts its navigation constant dynamically based on speed and distance.
   - Gyroscopes and thrusters are controlled to steer and manage speed.
6. **Detonation**:
   - The missile detonates warheads upon reaching the detonation distance or when specific conditions are met (e.g., sensor detection).

## Configuration Parameters

- **Navigation Constants**:
  - `_navConstant`: Adjusted dynamically; initial value can be set or modified.
  - `minNavConstant` and `maxNavConstant`: Bounds for the dynamic navigation constant.
- **Modes**:
  - `isTopdown`: Enables lofted trajectory for top-down attacks when set to `true`.
  - `_antiairmode`: Activates anti-air mode, allowing dynamic target updates.
- **Waypoints**:
  - Waypoints can be set through custom data or script arguments.
  - Lofted trajectories add intermediate waypoints for specific attack profiles.

## Potential Enhancements

- **Advanced Error Handling**: Implement robust checks and exception handling for all block accesses.
- **Modular Design**: Refactor into separate classes or methods for guidance, control, and user interface.
- **Performance Optimization**: Adjust update frequencies and optimize calculations for reduced computational load.
- **Enhanced Guidance Algorithms**: Integrate additional algorithms like Pure Pursuit or augmented PNG.
- **User Interface Improvements**: Add graphical elements or customizable displays on the LCD.

## Dependencies

- **Space Engineers ModAPI**: Utilizes in-game scripting API provided by Space Engineers.
- **C# Version**: Written in C# 6.0.
- **.NET Framework**: Targets .NET Framework 4.8.


This is an ai written summary - you shouldn't you this it will drive you mad. Its not written for public use. 
