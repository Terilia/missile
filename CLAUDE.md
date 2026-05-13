# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Space Engineers ingame script implementing a **Proportional Navigation (ProNav)** missile guidance system. The missile launches off a merge block ("Bay N") and receives target GPS via the launcher's `JETOS Programmable Block` Custom Data. Built with **MDK 2** targeting .NET Framework 4.8 / C# 6 so it can paste into a Programmable Block in-game.

## Build

```bash
dotnet build missile.sln
```

MDK 2 packagers produce the pastable script. Minification is controlled in `missile/missile.mdk.ini` (currently `minify=none`; use `lite`/`full` if script size becomes a problem). `missile/missile.mdk.local.ini` is git-ignored for local overrides. There is no test harness — validation requires running in Space Engineers.

## Code Layout

`Program` is a single `partial class` split across files by responsibility. When adding state, add fields to `MissileState.cs` — everything else assumes they live there.

- `Program.cs` — `Main()` only: the tick-driven state machine and the ProNav loop.
- `MissileState.cs` — all fields, constants (`WAYPOINT_THRESHOLD`, `DETONATION_DISTANCE`, `tickTime`), and `_navConstant`.
- `Initialization.cs` — `Initialize()`, `CheckForGPSAndStart()` (parses launcher Custom Data), `InitializeThrusters()`, `FindClosestMergeBlock()`, `GetBayNumberFromMergeBlock()`.
- `Guidance.cs` — `GyroTurn6()` (quaternion-based gyro steering, used by the live loop) and `ApplyGyroOverride()` (legacy/startup orientation helper).
- `Detonation.cs` — `PerformRaycastCheck()` (3 m ProxCam raycast) and `DetonateWarheads()`.
- `Waypoints.cs` — `TryParseGPS()` and `AddLoftedTrajectoryWaypoints()`.
- `VectorMath.cs` — `Vector_Projection_Scalar()` used for thrust alignment, plus a nested `VectorMath` static class with `Projection`/`Reject`.
- `Display.cs` — `DisplayOnLCD()` telemetry writer (runs every 3 ticks).

## Tick-Driven State Machine

`Runtime.UpdateFrequency = Update1` (every frame, 60 Hz). `Main()` branches on two flags set in order:

1. **`!_isInitialized`** — every 5 ticks, search own grid for a merge block whose name starts with "Bay", extract the trailing digits as `_bayNumber`, set `_isInitialized`, return.
2. **`_isInitialized && !_isStarted`** — `CheckForGPSAndStart()` reads `JETOS Programmable Block` Custom Data: parses `Topdown:`, `AntiAir:`, and the line starting with `<bayNumber>:`. On a valid GPS parse, seeds `_waypoints`, disables the merge block, resets `_ticks = 0`, sets `_isStarted`. In anti-air mode, the bay's GPS line is cleared from Custom Data so it won't be re-read.
3. **`_isStarted` tick 0–99** — staged hardware init. Block lookups happen in the 4 < tick < 10 window (remote control, radar, LCD, sensor, warheads, gyros, thrusters; thrusters are filtered by name containing `"Sci-Fi"` unless `armtype == "bomb"`). The gravity-aware uplift orientation runs 15 < tick < 30. `MissileMass` and `MissileThrust` are computed once at tick 20. Lofted waypoints (if `isTopdown`) are inserted at tick ~5–10.
4. **`_isStarted` tick ≥ 100** — full guidance loop, every frame.

The guidance loop lives in `Main()` (`Program.cs`) — it is not extracted into a method. Anti-air mode re-parses the launcher's `Cached:` GPS line every tick and rewrites the final waypoint.

## ProNav Guidance (Program.cs)

The current algorithm is **standard PN** — prior augmentation/drift-cancel terms were deliberately removed (see comments "No strange augmentation terms" and the disabled terminal-phase block).

```
Vclosing   = max(1.0, dot(MissileVelocity - TargetVelocity, LOS_New))
LOS_Rate   = |LOS_New - LOS_Old| / dt
Lateral    = LateralDir * _navConstant * LOS_Rate * Vclosing      // N · Vc · σ̇
```

Key details:
- `_navConstant` defaults to **4.0** in `MissileState.cs`, overridden to **9.0** when `armtype == "bomb"` at the top of `Main()`.
- `TargetVelocity` prefers radar `detectedEntity.Velocity`; falls back to finite-difference from `_previousTargetPoS`. Zero is only used if nothing is known.
- `LateralDirection` defaults to `normalize(LOS_Delta)`; degenerate case falls back to `Cross(Cross(LOS, RelVel), LOS)`.
- **Oversteer**: if `|Lateral| / MissileAccel > 0.98`, clamp to `0.98 * MissileAccel` in the commanded direction (no drift cancellation — this is a simpler clamp than older versions).
- Remaining accel budget is allocated along `LOS_New` via `sqrt(MissileAccel² - |Lateral|²)`.
- Final `desiredAcceleration = normalize(Lateral + LOS·rejected − gravity)` — gravity compensation is baked into the desired direction.
- There is a `_isTerminalPhase` branch for lead pursuit, but the switch that would set it is **commented out**. Treat terminal phase as dead code unless re-enabled.

**Gyro control** uses `GyroTurn6()` with **adaptive gain and damping** computed per-frame:

```
gainMultiplier  = min(1.0, distanceToTarget / 500.0)
adaptiveGain    = 18.0 * max(0.6, gainMultiplier)                 // floor raised to 60%
adaptiveDamping = 0.3 * (1.0 + 500.0 / max(distanceToTarget, 50.0))
```

`GyroTurn6()` converts the desired world vector into RC-local azimuth/elevation via inverse quaternion, applies PID damping against `PREV_Yaw`/`PREV_Pitch`, then transforms into each gyro's local frame. Outputs are clamped to `[-500, 500]`.

**Thrust modulation**: thrusters are no longer pinned at 100%. Each frame:

```
ThrustPower = clamp(dot(MissileForwards, normalize(Lateral)), 0.5, 1.0)
```

so thrust scales 50–100% by alignment with the commanded accel vector. `_remoteControl.DampenersOverride` is forced `false`.

## Target Tracking Modes

- **Standard** (`_antiairmode=false`) — static waypoint from launch.
- **Anti-Air** (`_antiairmode=true`, `armtype!="bomb"`) — every tick, re-parse `Cached:` line and overwrite final waypoint. Also drives the onboard `"Radar"` turret: fires `ShootOnce()` every 150-tick cooldown when no lock and within 6000 m; on lock, uses `detectedEntity.Position/Velocity`.
- **Topdown** (`isTopdown=true`) — `AddLoftedTrajectoryWaypoints()` inserts a waypoint at `0.5 * horizontalDistance` elevated by `9000 m` against gravity. Gravity-required; logs and skips in vacuum. Only runs when `!_antiairmode && armtype!="bomb"`.
- **Bomb** (`armtype="bomb"`) — `_navConstant=9.0`, radar/LCD skipped, thruster filter skipped (no `"Sci-Fi"` requirement), raycast detonation disabled.

## Block Naming (required for operation)

On the missile grid:
- `Remote Control Missile` — `IMyRemoteControl` (required; throws if missing)
- `Bay <N>` — `IMyShipMergeBlock`, digits parsed as bay number
- Thrusters named containing `Sci-Fi` — main propulsion (non-bomb mode)
- `Radar` — `IMyLargeGatlingTurret` (anti-air)
- `ProxCam` — `IMyCameraBlock` (optional proximity fuse)
- `Holo LCD` — `IMyTextPanel` (optional telemetry)
- `Sensor` — `IMySensorBlock` (enabled at init; no current consumer)

On the launcher grid:
- `JETOS Programmable Block` — target provider (required)

## Launcher Custom Data Format

```
Topdown:true
AntiAir:false
1:GPS:Target:X:Y:Z:#RRGGBB:
2:GPS:Target:X:Y:Z:#RRGGBB:
Cached:GPS:Live:X:Y:Z:#RRGGBB:
```

`TryParseGPS` does a `:`-split and reads parts `[2]`, `[3]`, `[4]` as X, Y, Z. No error handling beyond "skip line" — malformed entries silently fail.

## Detonation

Triggers:
1. `distanceToTarget <= DETONATION_DISTANCE` (8 m).
2. `distanceToTarget < 500` **and** `PerformRaycastCheck()` (3 m ProxCam ray hits something).
3. Manual: running the PB with argument `"detonate"`.

`DetonateWarheads()` lazily re-fetches warheads if the cached list is empty.

## Tuning Knobs (things to touch first)

- `_navConstant` (`MissileState.cs`) — pursuit aggressiveness. 4 for missile, 9 for bomb.
- `GAIN` base `18.0` and `DAMPINGGAIN` base `0.3` (in `Main()` before `GyroTurn6` call) — gyro response/damping. Both are now scaled adaptively by distance.
- `WAYPOINT_THRESHOLD` (550 m), `DETONATION_DISTANCE` (8 m) in `MissileState.cs`.
- Loft `fraction=0.5`, `loftHeight=9000` are `AddLoftedTrajectoryWaypoints()` default parameters.
- LCD cadence: every 3 ticks (`_ticks % 3 == 0`) in the loop tail.

## Constraints and Gotchas

- 60 Hz is load-bearing (`tickTime = 1/60`, `Global_Timestep = tickTime`). Do not change `UpdateFrequency`.
- All main thrusters are assumed co-aligned (forward). `_thrusters[0].WorldMatrix.Backward` is used as the missile-forward reference for thrust modulation.
- `Lofted waypoint` math assumes a single gravity source.
- Block searches are cached after the tick 4–10 init window; don't move lookups into the hot path.
- Multiple `Bay N` merge blocks on a grid → the closest one to `Me` wins. Keep names unique per missile.
- `_isTerminalPhase` currently unreachable — the enabling condition is commented out in `Main()`. Do not assume it runs.
