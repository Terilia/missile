# Hybrid AI Seeker Missile Refactor Design

## Goal

Refactor the missile script so one codebase supports both passive missiles and active missiles.

Passive missiles keep the current JetOS-fed guidance path: JetOS selects a target, streams position and velocity over IGC, and the missile flies the existing proportional-navigation guidance profile.

Active missiles add an optional onboard seeker made from one `AI Flight` block and one `AI Combat` block. If those blocks exist and produce a valid lock, the onboard seeker becomes the preferred target source. If they are missing, damaged, not locked, or stale, the missile continues using the JetOS stream.

The refactor must preserve the current guidance behavior as the baseline, while reducing per-tick calculation and removing legacy seeker work that is no longer needed.

## Current Context

The missile project is an MDK2 Space Engineers ingame script targeting .NET Framework 4.8 / C# 6. It is organized as a partial `Program` class:

- `missile/Program.cs` owns the tick state machine, staged startup, active guidance loop, target update logic, and control calls.
- `missile/MissileState.cs` owns fields, constants, cached block references, guidance state, IGC listener, and telemetry state.
- `missile/Initialization.cs` reads JetOS launch data, detects the bay, registers the per-bay IGC listener, and initializes thrusters.
- `missile/Guidance.cs`, `missile/Salvo.cs`, `missile/VectorMath.cs`, `missile/Waypoints.cs`, `missile/Detonation.cs`, and `missile/Display.cs` hold helper logic.

JetOS already has the radar behavior we want to port in focused form:

- `Mdk.PbScript2/Utilities/RadarTrackingModule.cs` wraps an `IMyFlightMovementBlock` plus `IMyOffensiveCombatBlock`.
- It reads the flight block waypoint list fed by the combat block, derives target velocity from recent positions, exposes target identity, and avoids using the flight block autopilot.
- `Mdk.PbScript2/Modules/RadarControlModule.cs` contains the larger multi-radar pool, RWR, menu, sound, and enemy-list orchestration. The missile should not port this whole module.

The current missile already supports an IGC target stream on `JETOS_MSL_<bayNumber>` and broadcasts status on `JETOS_MSL_STAT`. That is the right passive guidance backbone.

## Non-Goals

This refactor will not port JetOS HUD rendering, RWR, radar menus, target cycling, sound management, enemy deduplication lists, or multi-radar pool management into the missile.

This refactor will not create separate active and passive script branches.

This refactor will not change launcher bay selection semantics unless needed to pass additional optional seeker mode flags. The default should remain compatible with current JetOS launch behavior.

This refactor will not replace the missile's guidance law with a new system. The guidance path may be cleaned up so target position and velocity are explicit, but the baseline behavior stays proportional-navigation/APN style.

## Design Summary

Add a small target acquisition layer between raw sensors and guidance:

1. `TargetTrack` represents the best target known this tick.
2. `PassiveTargetProvider` updates from JetOS IGC and launcher CustomData fallback.
3. `ActiveAiSeeker` optionally updates from onboard `AI Flight` + `AI Combat`.
4. `TargetCoordinator` chooses the best track and updates the final waypoint.
5. The guidance loop consumes only the selected `TargetTrack`.

The target source priority is:

1. Active onboard AI seeker lock, when present and fresh.
2. JetOS IGC stream, when fresh.
3. Launcher `Cached:` CustomData fallback, when anti-air mode is enabled and no fresh IGC/seeker exists.
4. Launch-time GPS waypoint.

This gives two missile types from the same script:

- Passive missile: no onboard AI blocks. It flies from JetOS updates.
- Active missile: has `AI Flight` and `AI Combat`. It flies from JetOS until the onboard seeker has a valid lock, then uses that local lock.

## Block Naming

Required blocks stay unchanged:

- `Remote Control Missile`
- `Bay N`
- gyros
- main thrusters
- warheads

Optional active seeker blocks:

- `AI Flight`
- `AI Combat`

The script should also accept numbered names if useful later, but the first implementation should use one seeker pair only. If both exact names are present, create the seeker. If one block is missing, seeker mode is disabled and the missile runs passive.

Legacy optional blocks:

- `Radar` gatling turret should be removed from the target path.
- `Sensor` should be removed unless a future fuse mode uses it.
- `ProxCam` remains optional for close-range fuse raycast.
- `Holo LCD` remains optional, but telemetry should stay low-rate.

## Target Track Model

Introduce a compact struct stored in `MissileState.cs`:

```csharp
struct TargetTrack
{
    public Vector3D Position;
    public Vector3D Velocity;
    public long EntityId;
    public string Name;
    public TargetSource Source;
    public int UpdatedTick;
    public bool HasPosition;
    public bool HasVelocity;
    public bool HasLock;
}

enum TargetSource
{
    LaunchGps,
    CustomData,
    Igc,
    AiSeeker
}
```

Freshness rules:

- AI seeker track is fresh if it updated within 30 ticks and reports a nonzero target position.
- IGC track is fresh if it updated within 30 ticks.
- CustomData fallback is only used when neither seeker nor IGC is fresh.
- Launch GPS never expires, but it has no velocity and no lock.

The guidance loop should not directly read `detectedEntity`, `targetvelocity`, or scattered booleans. It should ask the coordinator for the selected track.

## Passive Target Provider

The passive provider keeps the current behavior but makes its output explicit:

- Drain the per-bay IGC listener each tick.
- Keep only the newest `MyTuple<Vector3D, Vector3D, Vector3D>` message.
- Store position, velocity, approach offset, source tick, and active flag.
- Read launcher `Cached:` CustomData only as fallback and only when the CustomData hash changes.
- Leave launch-time waypoint untouched if no live update exists.

This preserves current JetOS streaming while reducing string parsing in the hot path.

## Active AI Seeker

Port the useful subset of JetOS `RadarTrackingModule` into the missile project as `AiSeeker.cs`.

The seeker:

- Holds references to one `IMyFlightMovementBlock` and one `IMyOffensiveCombatBlock`.
- Keeps the flight block disabled and collision avoidance off.
- Enables/configures the combat block once during staged init.
- Applies the combat behavior activation once after the missile is live.
- Reads `flight.GetWaypoints(_waypointBuffer)` every tick.
- Tracks the last two target positions with timestamps based on `Runtime.TimeSinceLastRun.Ticks`.
- Derives target velocity only after two valid samples.
- Caches target name parsing and updates it only when `FoundEnemyId` changes.
- Returns a `TargetTrack` with `Source = AiSeeker` when a valid lock exists.

The missile does not need JetOS's sequential multi-radar state machine. A single active seeker has only two states:

- `Inactive`: missing blocks or disabled by config.
- `Seeking`: blocks exist and combat behavior is active; track may or may not be locked this tick.

The seeker must fail soft. If either AI block is missing, closed, destroyed, or reports no target, the missile continues passive guidance.

## Guidance Integration

Guidance should remain structurally familiar:

- Waypoint navigation stays for launch GPS and topdown paths.
- On the final leg, selected target position updates the final waypoint.
- Target velocity from the selected track flows into intercept and APN guidance.
- Target acceleration estimation resets when the selected target source changes or when velocity is unavailable.
- Gravity compensation remains.
- Gyro and thrust application remain the final control stages.

The current loop computes an intercept aim point with target velocity, then calls `ComputeGuidance(..., Vector3D.Zero, ...)`. The refactor should make that handoff explicit:

- Use selected track position/velocity for direct final-leg guidance.
- If using an intercept aim point, either pass the predicted target velocity deliberately or treat the aim point as a static waypoint and reset APN acceleration for that tick.
- Avoid accidentally using stale velocity after lock loss.

The intended behavior is:

- Cruise/loft waypoints use zero target velocity.
- Final-leg passive IGC uses JetOS target velocity when fresh.
- Final-leg active seeker uses onboard AI-derived velocity when locked.
- Lock loss resets target acceleration history.

## Calculation Cost Reductions

Move or remove work that currently runs more often than needed:

- Remove gatling turret `radar.GetTargetedEntity()` and `ShootOnce()` logic.
- Remove `Radar` block state from `MissileState.cs`.
- Remove `Sensor` lookup and enable unless retained by a later fuse feature.
- Cache missile mass after staged init and refresh it at a low rate, not every tick.
- Cache max effective thrust after staged init and refresh at a low rate or only when a thruster is nonfunctional.
- Keep IGC draining every tick, but parse launcher CustomData only on hash change.
- Keep LCD updates at 20 Hz or lower; do not build display strings every tick.
- Reuse seeker waypoint buffers.
- Avoid repeated `GetBlocksOfType` calls after staged init except warhead lazy fallback.
- Avoid repeated block property writes unless a value changed or the block was just initialized.

The active seeker does add one `GetWaypoints()` call per tick when present. That cost replaces the old turret radar work and only exists on active missile designs.

## Error Handling

Required block failures remain hard failures during staged init:

- Missing remote control should still throw.
- Missing main thrusters should still prevent launch guidance from continuing.

Optional block failures are soft:

- Missing AI seeker pair: passive mode.
- Partial AI seeker pair: passive mode and one throttled `Echo`.
- AI seeker lock lost: fall back to IGC if fresh, otherwise last valid passive/launch target.
- Missing LCD: no display.
- Missing ProxCam: no raycast fuse.

The coordinator should record source changes for telemetry and debugging, but it should not spam `Echo` every tick.

## Telemetry

Telemetry should show the selected source in compact form:

- `SRC GPS`
- `SRC IGC`
- `SRC AI`
- `SRC CD`

`SendStatus()` should keep using the existing acquired flag, but it should mean "has live target source" rather than only "radar lock or IGC." Active seeker lock counts as acquired.

LCD target name should use the selected track name when available.

## Compatibility

Existing passive missiles should continue to work without adding AI blocks.

Existing JetOS launch behavior should continue to work:

- `Topdown`
- `AntiAir`
- per-bay GPS slots
- `JETOS_MSL_<bayNumber>` target stream
- `JETOS_MSL_STAT` status stream

Existing active missile blueprints that use a gatling turret named `Radar` will no longer get terminal seeker behavior from that turret after the refactor. They need the AI seeker pair to become active missiles.

## Testing And Verification

There is no automated in-game test harness. Verification should combine build checks with focused code-level seams:

1. Run `dotnet build missile.sln` after each refactor stage.
2. Keep target selection logic in small methods that can be inspected and reasoned about independently.
3. Use compile-time checks to catch Space Engineers API type issues.
4. Validate passive mode in-game with no AI blocks: launch, receive JetOS IGC updates, guide to target, status telemetry shows IGC/GPS.
5. Validate active mode in-game with AI blocks: launch, seeker locks, source changes to AI, target velocity updates, lock loss falls back to IGC.
6. Validate degraded active mode: only one AI block present, damaged AI blocks, missing ProxCam, missing LCD.
7. Validate cost reduction through `Runtime.CurrentInstructionCount` logging before and after active guidance.

## Implementation Boundaries

The refactor should be staged to avoid breaking guidance while moving target acquisition:

1. Add `TargetTrack` and coordinator while keeping old behavior equivalent.
2. Move passive IGC/CustomData logic behind the coordinator.
3. Port the single-pair AI seeker and wire it as optional.
4. Remove legacy gatling radar and sensor state.
5. Clean the hot loop for mass/thrust refresh and display/string cadence.
6. Update docs and build.

Each stage should compile before moving to the next.
