# ProNav Missile

A proportional navigation missile guidance system for Space Engineers programmable blocks. Provides merge-block launched missiles with GPS targeting, lofted trajectories, anti-air radar tracking, proximity detonation, and LCD telemetry. Designed to work with [JetOS](https://github.com/Terilia/JetOS). Built with [MDK2](https://github.com/malware-dev/MDK2).

## Features

- **Proportional Navigation** — True PN guidance with adaptive gain, PID-damped gyro control, and gravity compensation
- **Lofted Trajectory** — Top-down attack mode with 9 km altitude intermediate waypoint for plunging fire
- **Anti-Air Tracking** — Continuous target updates from launcher CustomData with onboard radar refinement
- **Radar Integration** — Gatling turret radar for terminal guidance, automatic target velocity extraction
- **Proximity Detonation** — Camera raycast proximity fuse (3 m) with distance-based backup (8 m)
- **LCD Telemetry** — Real-time missile cam display with speed, distance, ETA, progress bar, and impact warning
- **Bomb Mode** — Alternative guidance profile with higher navigation constant for gravity weapons
- **Multi-Bay Support** — Bay number auto-detection from merge block names, supports parallel launches

## Requirements

- [MDK2](https://github.com/malware-dev/MDK2) (Malware's Development Kit 2)
- .NET Framework 4.8 / C# 6.0
- Space Engineers (PC)

## Build

```bash
dotnet build missile.sln
```

There are no automated tests — verification is done in-game. Minification level is configured in `missile/missile.mdk.ini`.

## Project Structure

```
missile/
├── Program.cs              Entry point (constructor, Main, guidance, all methods)
├── missile.csproj           C# project file
├── missile.mdk.ini          MDK2 configuration (minification, packaging)
├── missile.mdk.local.ini    Local overrides (git-ignored)
├── Instructions.readme      MDK2 injected header
└── thumb.png               Workshop thumbnail
```

Single-file script — all logic lives in `Program.cs` as `partial class Program` inside the `IngameScript` namespace. MDK2 compiles it for Space Engineers' ingame scripting environment.

## In-Game Setup

### Required Blocks (Missile Grid)

| Block | Name | Purpose |
|-------|------|---------|
| Programmable Block | *(this script)* | Runs missile guidance logic |
| Remote Control | `Remote Control Missile` | Orientation reference and gravity/velocity APIs |
| Merge Block | `Bay X` (e.g., `Bay 1`) | Launcher attachment point, bay number auto-detected from name |
| Gyroscope(s) | *(any name)* | Steering — all gyros on grid are used |
| Thruster(s) | Contains `Sci-Fi` | Main propulsion (filtered by name in missile mode) |
| Warhead(s) | *(any name)* | Detonation payload |

### Optional Blocks (Missile Grid)

| Block | Name | Purpose |
|-------|------|---------|
| Text Panel | `Holo LCD` | Telemetry display (speed, distance, ETA, progress) |
| Camera | `ProxCam` | Proximity detonation raycast (3 m range) |
| Gatling Turret | `Radar` | Anti-air target detection and velocity extraction |
| Sensor | `Sensor` | Legacy block (enabled on startup) |

### Launcher Grid

| Block | Name | Purpose |
|-------|------|---------|
| Programmable Block | `JETOS Programmable Block` | Provides target GPS and mode flags via CustomData |
| Merge Block(s) | `Bay X` | Corresponding missile attachment points |

Thrusters with `"Sci-Fi"` in the name are used as main propulsion. In bomb mode, all thrusters are used regardless of name.

## Custom Data Format

The launcher's Programmable Block CustomData must contain:

```
Topdown:true
AntiAir:false
1:GPS:Target:12345.6:67890.1:-23456.7:#FF0000:
2:GPS:Target:98765.4:43210.9:-87654.3:#00FF00:
Cached:GPS:LiveTarget:11111.1:22222.2:33333.3:#0000FF:
```

| Key | Format | Purpose |
|-----|--------|---------|
| `Topdown` | `true` / `false` | Enable lofted trajectory (9 km altitude waypoint) |
| `AntiAir` | `true` / `false` | Enable continuous target updates from `Cached:` line |
| `<BayNumber>:GPS:...` | SE GPS format | Static target assignment per bay |
| `Cached:GPS:...` | SE GPS format | Live target position (anti-air mode, updated by launcher) |

## Documentation

Detailed system documentation with diagrams lives in [`docs/`](docs/):

| Document | Contents |
|----------|----------|
| [Architecture](docs/architecture.md) | Execution phases, initialization order, tick timeline, active guidance loop |
| [Guidance](docs/guidance.md) | ProNav algorithm, LOS rate, oversteer correction, GyroTurn6, thrust modulation |
| [Targeting](docs/targeting.md) | GPS parsing, anti-air updates, radar acquisition, launcher communication |
| [Flight Modes](docs/flight-modes.md) | Mode selection, waypoint navigation, lofted trajectory, bomb mode, initial orientation |
| [Detonation](docs/detonation.md) | Decision tree, proximity raycast, warhead sequence, conditions timeline |
| [Telemetry](docs/telemetry.md) | LCD update pipeline, display layout, impact warning animation |

## Architecture

The missile operates as a single-file state machine with four execution phases:

1. **Initialization** (ticks 0-5) — finds merge block, extracts bay number
2. **Waiting for Launch** — reads target GPS from launcher CustomData, waits for merge disconnect
3. **Early Startup** (ticks 5-100) — initializes blocks, orients toward target, calculates mass/thrust, inserts lofted waypoints
4. **Active Guidance** (ticks 100+) — full ProNav loop at 60 Hz: anti-air updates → radar tracking → detonation checks → PN acceleration → gyro steering → thrust modulation → LCD telemetry

The guidance core uses standard Proportional Navigation (`a = N * Vc * LOS_rate`) with adaptive gain that reduces near the target to prevent oscillation, PID-damped gyro control via quaternion reference frame transforms, and alignment-based thrust modulation (50-100%).

## Debugging

```csharp
Echo($"Debug: {value}");
Echo($"Instructions: {Runtime.CurrentInstructionCount}");
```

The script runs at `UpdateFrequency.Update1` (every tick). Tick counter is displayed via `Echo(_ticks.ToString())`.

## License

Private repository.
