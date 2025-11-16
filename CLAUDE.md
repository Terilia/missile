# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a **Space Engineers ingame script** project that implements a **Proportional Navigation (ProNav) missile guidance system**. The missile script is designed to work with merge-block launched missiles that receive target GPS coordinates from a controlling script (typically "JETOS Programmable Block").

The project uses **MDK 2 (Malware Development Kit)** - a Visual Studio extension and toolchain for developing Space Engineers scripts in C# 6 targeting .NET Framework 4.8.

## Build and Development Commands

### Building the Script
```bash
dotnet build missile.sln
```

The build process uses MDK 2 packagers that compile the script and prepare it for deployment to Space Engineers. The output is optimized for the game's ingame scripting environment.

### Project Configuration
- Target Framework: .NET Framework 4.8
- Language: C# 6
- Platform: x64
- MDK Configuration: `missile/missile.mdk.ini`
- Local overrides (git-ignored): `missile/missile.mdk.local.ini`

### Minification Settings
Controlled in `missile/missile.mdk.ini`:
- Currently set to `minify=none` (no minification)
- Options: `none`, `trim`, `stripcomments`, `lite`, `full`
- For production deployment, consider using `lite` or `full` to reduce script size

## Architecture and Code Structure

### Missile Launch and Initialization Flow

1. **Pre-Launch State**: Missile sits on launcher, attached via merge block named "Bay X" where X is the bay number
2. **Initialization** (Main loop, `!_isInitialized`):
   - Finds closest merge block with "Bay" in name on own grid
   - Extracts bay number from merge block name (e.g., "Bay 1" → `_bayNumber = 1`)
   - Sets `_isInitialized = true`
3. **Waiting for Launch** (`_isInitialized && !_isStarted`):
   - Calls `CheckForGPSAndStart()` which:
     - Reads Custom Data from "JETOS Programmable Block" on launcher grid
     - Parses `Topdown:true/false` flag (enables lofted trajectory)
     - Parses `AntiAir:true/false` flag (enables continuous target updates)
     - Searches for line matching bay number format: `<BayNumber>:GPS:Target:X:Y:Z:#Color:`
     - Once GPS found, populates `_waypoints`, disconnects merge block, sets `_isStarted = true`
4. **Active Guidance** (`_isStarted`):
   - Ticks 0-100: Initialization phase (find blocks, enable systems, calculate lofted waypoints if topdown mode)
   - Tick 100+: Full guidance loop runs every frame (60 FPS)

### Guidance Algorithm (PN with Augmentation)

The core guidance is in the Main loop starting at line 421. Key calculations:

1. **Line-of-Sight Rate**: Calculates instantaneous LOS rate using cross product: `Cross(targetdirection, (currentVelocity - targetvelocity)) / targetdirection.LengthSquared()`

2. **Lateral Acceleration**:
   ```
   LateralAccelerationComponent = LateralDirection * 5 * LOS_Rate * Vclosing + LOS_Delta * 9.8 * (0.5 * 5)
   ```
   This is a PN law with navigation constant of 5.

3. **Oversteer Correction**: If lateral acceleration exceeds 98% of missile's max acceleration, the algorithm adds drift cancellation to maintain minimum intercept time.

4. **Axial Acceleration**: Remaining thrust after lateral maneuver is projected along LOS for closure.

5. **Gyro Control**: `GyroTurn6()` function (line 509) converts desired acceleration vector to gyro pitch/yaw/roll commands with PID damping.

### Target Tracking Modes

**Standard Mode** (`_antiairmode = false`):
- Target is static GPS coordinate from launcher's Custom Data
- Waypoints fixed at launch time

**Anti-Air Mode** (`_antiairmode = true`, `armtype != "bomb"`):
- Continuously reads `Cached:GPS:` line from launcher's Custom Data
- Updates final waypoint each frame to track moving targets
- Uses onboard radar (IMyLargeGatlingTurret named "Radar") to detect and refine target position
- If radar locks target, uses `detectedEntity.Position` and `detectedEntity.Velocity` for guidance

**Topdown Mode** (`isTopdown = true`, set via launcher Custom Data):
- Calls `AddLoftedTrajectoryWaypoints()` once after initialization
- Inserts intermediate waypoint at fraction (default 0.5) of horizontal distance, elevated by `loftHeight` (default 9000m)
- Missile climbs to loft point, then descends onto target

**Bomb Mode** (`armtype = "bomb"`):
- Sets higher navigation constant (`_navConstant = 9.0`)
- Disables radar and LCD
- Different thruster filtering logic (skips "Sci-Fi" named thrusters)

### Block Naming Conventions (Critical for Operation)

The script expects specific block names on the missile grid:
- `"Remote Control Missile"` - IMyRemoteControl (required)
- `"Bay X"` - IMyShipMergeBlock where X is bay number (required, e.g., "Bay 1", "Bay 2")
- `"Sci-Fi"` - Thrusters containing this substring are used for main propulsion (not in bomb mode)
- `"Radar"` - IMyLargeGatlingTurret for target detection (anti-air mode)
- `"ProxCam"` - IMyCameraBlock for proximity detonation (optional)
- `"Holo LCD"` - IMyTextSurface for status display (optional, not in bomb mode)
- `"Sensor"` - IMySensorBlock (required but may be legacy/unused)

On the launcher grid:
- `"JETOS Programmable Block"` - IMyProgrammableBlock that provides target GPS via Custom Data

### Custom Data Format (Launcher → Missile Communication)

The launcher's Programmable Block Custom Data must contain:
```
Topdown:true
AntiAir:false
1:GPS:Target:12345.6:67890.1:-23456.7:#FF0000:
2:GPS:Target:98765.4:43210.9:-87654.3:#00FF00:
Cached:GPS:LiveTarget:11111.1:22222.2:33333.3:#0000FF:
```

- `Topdown:` - Boolean flag for lofted trajectory
- `AntiAir:` - Boolean flag for continuous target updates
- `<BayNumber>:GPS:...` - Target assignment per bay
- `Cached:GPS:...` - Live target position (anti-air mode only)

After the missile reads its target, it clears its bay line in anti-air mode to prevent re-reads.

### Detonation Logic

Warheads detonate when:
1. Distance to final waypoint ≤ `DETONATION_DISTANCE` (8.0m)
2. Proximity camera raycast hits object within 3m (if distance < 500m)
3. Manual trigger via `"detonate"` argument to programmable block

### Thrust Management

Full thrust (100%) is applied to all main thrusters once guidance starts (line 491). Earlier iterations had dynamic thrust logic but current version uses constant full thrust. The missile relies on gyro steering for maneuverability.

## Important Implementation Notes

### Vector Math Utilities
- Custom `VectorMath` class (line 818) provides `Projection()` and `Reject()` for vector operations
- Used in lofted trajectory calculations to separate horizontal/vertical components

### Frame Rate Assumptions
- Script assumes 60 FPS (`tickTime = 1f / 60f`, `Global_Timestep = 0.016`)
- Constructor sets `Runtime.UpdateFrequency = UpdateFrequency.Update1` (every frame)
- This is critical for guidance accuracy - do not change update frequency

### Coordinate System Transformations
The guidance algorithm transforms vectors between multiple reference frames:
1. World space (GPS coordinates)
2. Remote Control reference frame
3. Individual gyro reference frames

`GyroTurn6()` performs these transformations using quaternions for precision.

### Performance Considerations
- All block searches (`GridTerminalSystem.GetBlocksOfType`) are cached after first tick
- LCD updates and radar operations only occur after tick 100 to reduce initialization lag
- String parsing of Custom Data happens only once at launch (except in anti-air mode)

## Common Development Scenarios

### Adjusting Guidance Aggressiveness
- Change `_navConstant` (line 81, default 5.0) - higher values = more aggressive pursuit
- Modify GAIN parameter in `GyroTurn6()` call (line 476, default 18) - higher = faster rotation response
- Adjust DAMPINGGAIN in `GyroTurn6()` call (line 476, default 0.3) - higher = more damping, reduces oscillation

### Adding New Waypoint Behaviors
- Waypoints are stored in `List<Vector3D> _waypoints`
- Current waypoint index is `_currentWaypointIndex`
- Waypoint switching logic at line 353: switches when within `WAYPOINT_THRESHOLD` (550m)
- Insert new waypoints before final target to create flight paths

### Modifying Detonation Behavior
- Change `DETONATION_DISTANCE` constant (line 79, default 8.0m)
- Modify proximity raycast distance in `PerformRaycastCheck()` (line 638, default 3.0m)
- Add additional detonation conditions in Main loop around line 408-418

### Debugging and Telemetry
- LCD display function `DisplayOnLCD()` (line 992) shows extensive telemetry
- Includes distance, velocity, time-to-impact, progress bar, scrolling quotes
- Special "explosion" animation when time-to-impact < 2.5 seconds
- Can add custom telemetry by modifying the output string composition

## Known Limitations and Edge Cases

1. **Single Thruster Direction**: Assumes all main thrusters point in same direction (forward). Multi-directional thrust vectoring not supported.

2. **Gravity Assumption**: Lofted trajectory assumes single gravity source. May behave incorrectly near multiple planets/moons.

3. **Target Velocity Estimation**: In non-anti-air mode, target velocity is assumed zero. For moving targets without radar, use anti-air mode.

4. **Merge Block Detection**: If multiple merge blocks with "Bay" in name exist, script chooses closest to programmable block. Ensure unique naming.

5. **Custom Data Parsing**: Simple string splitting, no error handling for malformed GPS. Invalid data causes mission failure.

6. **Radar Lock Loss**: If radar loses lock in anti-air mode, missile continues to last known position. No re-acquisition logic beyond periodic shots.

## Testing Workflow

Since this is an ingame script, testing requires Space Engineers:

1. Build the script with `dotnet build`
2. Copy output from build directory (or use MDK auto-deploy if configured)
3. Paste into Programmable Block in Space Engineers
4. Set up test missile with required blocks (see Block Naming Conventions)
5. Set up launcher with JETOS Programmable Block and correct Custom Data format
6. Trigger merge block disconnect to launch

For code-only validation without game:
- Syntax checking via build process
- No unit test framework currently exists
- Logic validation requires in-game testing
