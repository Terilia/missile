# Missile Script Code Review — Findings

Generated review of the current codebase (`missile/`) after the refactor into partial-class files. Grouped by severity. Logic is preserved in the notes — these are observations, not proposed simplifications.

Cross-referenced against **Whip's SpaceEngineersScripts** (`../SpaceEngineersScripts-master/`) as the SE-math gold standard: `Released/WHAM.cs` (Whip's Homing Advanced Missile), `Classes/PID.cs`, `Classes/VectorMath.cs`, `Unpolished/ApplyGyroOverride.cs`, plus the `GetRotationVector` helper at `WHAM.cs:4265`. Each finding below is tagged **WHAM-confirmed** where the reference disagrees with the current missile code, or **WHAM-differs** where both approaches are valid but Whip's is numerically cleaner.

## Real bugs (change behavior)

### 1. Thruster filter is dead code
`Program.cs:180–198`
```csharp
List<IMyThrust> filteredThrusters = new List<IMyThrust>();
foreach (var thruster in _thrusters)
    if (thruster.CustomName.Contains("Sci-Fi"))
        filteredThrusters.Add(thruster);
if (filteredThrusters.Count == 0) { Echo("No thrusters found!"); return; }
InitializeThrusters();   // operates on _thrusters, NOT filteredThrusters
```
`filteredThrusters` is built, guarded on, then discarded. `InitializeThrusters()` uses the raw `_thrusters` list. Downstream effects:
- Every thruster on the grid (including sideways RCS) gets `ThrustOverridePercentage = 1f`.
- The main-loop thrust modulation at `Program.cs:507–515` applies `MaxThrust * ThrustPower` to every thruster, regardless of facing.
- `_thrusters[0].WorldMatrix.Backward` being used as "missile forward" is now fragile — `[0]` might be sideways.

### 2. Variable shadow in `ApplyGyroOverride`  **WHAM-confirmed**
`Guidance.cs:98`
```csharp
if (roll1 <= 90 || roll >= 270)   // `roll` is float Z-angular-velocity, NOT the degrees roll1
    upsidedown = -1;
```
`roll` is `localAngularVelocity.Z` in rad/s. The `270` compare was clearly intended for `roll1` (degrees). The upside-down flag almost never flips as intended.

### 3. Gyro axis mapping in `ApplyGyroOverride`  **WHAM-confirmed**
`Guidance.cs:107–109`
```csharp
gyro.Pitch = -pitch;   // pitch  = localAngularVelocity.Y
gyro.Yaw   = roll;     // roll   = localAngularVelocity.Z -> Yaw
gyro.Roll  = yaw;      // yaw    = localAngularVelocity.X -> Roll
```
Compare against Whip's canonical `ApplyGyroOverride` (`Unpolished/ApplyGyroOverride.cs:7–21`):
```csharp
var rotationVec = new Vector3D(pitchSpeed, yawSpeed, rollSpeed);    // X=pitch, Y=yaw, Z=roll
var relativeRotationVec = Vector3D.TransformNormal(rotationVec, worldMatrix);          // world -> ship frame
foreach (var g in gyroList) {
    var v = Vector3D.TransformNormal(relativeRotationVec, Matrix.Transpose(g.WorldMatrix)); // -> gyro frame
    g.Pitch = (float)v.X;
    g.Yaw   = (float)v.Y;
    g.Roll  = (float)v.Z;
}
```
Two SE-specific rules the current code breaks:
- **X/Y/Z → Pitch/Yaw/Roll**, not permuted. The current code takes `.Y` as pitch, `.X` as yaw, `.Z` as roll, then writes them back in yet another order.
- **World → ship → gyro**, two transforms. Current code goes world → gyro directly, which loses the reference-frame step that makes multi-gyro setups with mixed orientations work. `WHAM.cs:4240–4244` explicitly documents Keen's convention: *pitch = −X rotation, yaw = −Y rotation, roll = −Z rotation*.

### 3a. Only used during startup, but the same pattern exists in `GyroTurn6` via azimuth/elevation  **WHAM-differs**
`Guidance.cs:GyroTurn6` uses `Vector3D.GetAzimuthAndElevation` on the inverse-quaternion-rotated direction vector. That gives correct pitch/yaw **for a body-frame reference vector** but has no roll constraint — so roll is computed from the PID derivative output and pushed straight through. Whip uses `GetRotationVector` (`WHAM.cs:4265`) which constructs a target orientation from *both* forward and up vectors, then extracts axis-angle via the matrix-trace formula (`WHAM.cs:4304–4309`). That gives the minimum-rotation PYR command with a proper roll reference, and it does not suffer from gimbal lock near forward = ±up. Current code is the "simple case" of Whip's at `WHAM.cs:4286–4287`.

### 4. Null check fires after first use
`Program.cs:177` uses `_remoteControl.GetPosition()` before `Program.cs:204`'s `if (_remoteControl == null) throw`. A missing RC crashes at 177 before the guard runs.

### 5. `targetvelocity` is never cleared on radar lock loss
Once the radar has ever locked, `targetvelocity` retains its last measured value forever. `Program.cs:415–417` keeps feeding the stale vector into PN:
```csharp
Vector3D TargetVelocity = (targetvelocity.LengthSquared() > 0) ? targetvelocity : ...;
```

### 6. `MissileThrust` uses nameplate `MaxThrust`  **WHAM-confirmed**
`Program.cs:256`. For atmospheric and ion thrusters this diverges from reality at altitude. `MissileAccel = MissileThrust / Mass` drives the oversteer clamp and the rejected-acceleration budget, so both are wrong in the wrong environment.

WHAM does it right (`WHAM.cs:2064–2067`):
```csharp
foreach (var block in mainThrusters)
    thrust += block.IsFunctional && !block.Closed ? block.MaxEffectiveThrust : 0;
```
Not `MaxThrust` — `MaxEffectiveThrust`. Also skips non-functional / closed blocks so a damaged missile recomputes a correct budget. And it's called inside `Navigation()` every update, not once at tick 20.

### 7. `MissileMass` frozen at tick 20  **WHAM-confirmed**
`Program.cs:247–253`. Current code sums `block.Mass` across all terminal blocks — this excludes inventory mass and construction inventory, and doesn't match what the physics engine uses.

WHAM (`WHAM.cs:1799`): `missileMass = _missileReference.CalculateShipMass().PhysicalMass;` — authoritative. Called every update.

### 8. `"detonate"` argument does not early-return
`Program.cs:82–85` — calls `DetonateWarheads()`, then continues executing the rest of Main for that tick.

### 9. Detonation tunneling
`DETONATION_DISTANCE = 8 m`. At typical missile speed 400–600 m/s, a 60 Hz step is 6.7–10 m — the final frame can step past the threshold without triggering. Fix with a segment-vs-sphere test, or compare previous-frame distance against current to detect crossing.

### 10. `radar == null` leaves `detectedEntity` stale
`Program.cs:317` only enters the radar block if `radar != null`. In bomb mode `radar` is always null, and `detectedEntity` is a struct default. But in non-bomb mode if the "Radar" block is missing, `detectedEntity` can be left with any previously-written data. `detectedEntity = default(MyDetectedEntityInfo)` on the null-radar branch would be safer.

## Math / logic issues (keep the logic, flag the quirk)

### 11. Gravity subtracted after accel budget is spent  **WHAM-confirmed**
`Program.cs:475–476`
```csharp
LateralAccelerationComponent = LateralAccelerationComponent + LOS_New * RejectedAccel;
desiredAcceleration = Vector3D.Normalize(LateralAccelerationComponent - gravity);
```
`LateralAccelerationComponent` was built so that `|Lateral|² + RejectedAccel² = MissileAccel²`. Subtracting world gravity can push the requested vector past the physical accel limit — the oversteer check already happened.

WHAM's gravity compensation (`WHAM.cs:2868–2880`, inside `GuidanceBase.GravityCompensation`):
```csharp
Vector3D directionNorm = VectorMath.SafeNormalize(desiredDirection);
Vector3D gravityCompensationVec = -(VectorMath.Rejection(gravity, desiredDirection));   // only the LATERAL part of gravity needs cancelling
double diffSq = missileAcceleration * missileAcceleration - gravityCompensationVec.LengthSquared();
if (diffSq < 0)                                   // not enough thrust to hover
    return desiredDirection - gravity;            // graceful degrade: accept we'll sink
return directionNorm * Math.Sqrt(diffSq) + gravityCompensationVec;
```
Three things the current code misses:
1. Only the component of gravity **perpendicular to the desired direction** needs to be cancelled by steering — the parallel component is budgeted as part of the axial closure, not as a steering error. `Rejection(gravity, desired)` isolates that perpendicular component. The current code subtracts full gravity, steering for the parallel component unnecessarily.
2. The remaining accel budget for the along-direction term is `sqrt(accel² − |gravity⊥|²)`, not the earlier `sqrt(accel² − |Lateral|²)`. Current code mixes both budgets and then normalizes away the magnitude.
3. Handles the "can't hover" case explicitly (low-thrust atmosphere) instead of producing a garbage vector.

### 12. Thrust alignment vector disagrees with steering vector
`Program.cs:500–503`
```csharp
Vector3D MissileForwards = _thrusters[0].WorldMatrix.Backward;
double ThrustPower = Vector_Projection_Scalar(MissileForwards, Vector3D.Normalize(LateralAccelerationComponent));
```
Gyros steer to `desiredAcceleration = Normalize(Lateral - gravity)`. Thrust is modulated against `Normalize(Lateral)` (no gravity). In gravity these point differently, so steering alignment and thrust alignment disagree.

### 13. `Vclosing` floor of 1.0 hides divergence  **WHAM-differs**
`Program.cs:427–428`. If the missile is not closing, PN still produces lateral command based on a fictional 1 m/s.

WHAM avoids the question entirely by using the vector omega form — closing speed never appears as a scalar that needs clamping. See the next finding for the exact formulation.

### 14. Whole PN formulation is the chord-length version, not vector-omega  **WHAM-confirmed**
`Program.cs:440–462` reconstructs `LOS_Rate = |LOS_New − LOS_Old| / dt` and a separate `LateralDirection = Normalize(LOS_Delta)` from scratch every frame, then multiplies `LateralDir × N × LOS_Rate × Vclosing`. This is the classical *scalar* PN formulation and it has real problems in SE:

- **Chord vs arc**: for two unit vectors at angle θ, `|LOS_New − LOS_Old| = 2 sin(θ/2)`. At θ ≈ 1 rad/tick (tight close-range pass) this underestimates the true angular rate by ~15%.
- **Direction noise**: `Normalize(LOS_Delta)` is unstable when the LOS change is tiny relative to floating-point precision. The current code has an `1e-10` threshold, but the Cross-product fallback below it is only correct as long as relative velocity exists — it fails identically at the degenerate moment where LOS actually isn't changing.
- **Needs a separate closing-speed term** that the clamp has to paper over.

WHAM's ProNav (`WHAM.cs:2955–2964`):
```csharp
Vector3D omega = Vector3D.Cross(missileToTarget, relativeVelocity)
                 / Math.Max(missileToTarget.LengthSquared(), 1);   // instability guard at close range
return NavConstant * relativeVelocity.Length()
       * Vector3D.Cross(omega, missileToTargetNorm)
     + NavAccelConstant * lateralTargetAcceleration;               // APN: maneuvering-target term
```
`omega = R × V_rel / R²` is the exact vector LOS rotation rate — no chord approximation, no sign ambiguity, no divide-by-zero (the floor on `|R|²` handles the terminal singularity). `|V_rel|` as the speed multiplier is always ≥ 0. `Cross(omega, LOS_norm)` gives the lateral direction directly — the current code's "normalize the LOS_Delta, fall back to double-cross" chain is a fragile reconstruction of that same vector.

Also note the `+ NavAccelConstant * lateralTargetAcceleration` term — that's **Augmented Proportional Navigation** against maneuvering targets. The current code ignores target acceleration entirely. `GuidanceBase.Update` finite-differences `targetVelocity` to estimate `targetAcceleration` (`WHAM.cs:2854–2857`), then the lateral component is projected and fed into the guidance.

### 15. Damping is an ad-hoc PD mixed into the error signal  **WHAM-confirmed**
`adaptiveDamping = 0.3 * (1 + 500/max(d, 50))` → 3.3 at d = 50 m. Then `Guidance.cs:36`:
```csharp
ShipForwardAzimuth = ShipForwardAzimuth + DAMPINGGAIN * ((ShipForwardAzimuth - YawPrev) / tickTime);
```
Mathematically this is `P + D·derivative(P)` with P=1, D=Kd, which *is* a valid PD controller — but:
- The derivative term is **added into the error variable** before it is clamped and converted to a gyro command downstream. That's not how a PID output is meant to be consumed; the rest of the code treats the mutated `ShipForwardAzimuth` as if it were still an error signal.
- The ratio D / (P·dt) = 0.3 / (1·(1/60)) = 18 means at d ≤ 50 m the derivative term can dominate the error term by a factor of ~60× the per-tick delta, producing sign-flip commands that chatter.
- There is no integral term, no anti-windup, no reset on target change.

Whip's `PID` class (`Classes/PID.cs`) is the SE-standard: keeps `_errorSum`, `_lastError`, `_firstRun` per-instance, offers clamped / decaying / buffered integral variants, and returns a single scalar `Value = Kp·err + Ki·∫err + Kd·d(err)/dt`. Used in WHAM as one PID per axis (pitch/yaw/roll). The angle command gets a separate `rotationSpeedPYR` from an angle-controller PID (`WHAM.cs:1949–1952`), and gyros get `ApplyGyroOverride(rotationSpeedPYR)`. Clean separation; no double-dipping.

### 16. Multi-gyro loop shares `PREV_Yaw`/`PREV_Pitch`  **WHAM-confirmed**
`Program.cs:488–491`. All gyros share the same previous-error scalars. Last-write-wins on update. If gyros are mounted in different orientations the damping derivative is meaningless.

Whip's pattern: one PID per axis (not per gyro), computed once on the *ship* frame, then `ApplyGyroOverride(rotVec, gyros, shipWorldMatrix)` re-projects the single ship-frame rotation command into each gyro's local frame separately. That way N gyros pulling in different rigging directions still apply the same *ship-frame* rotation rate — no shared PID state, no per-gyro drift.

### 17. `LateralDirection` noisy on near-collinear LOS
`Program.cs:446–458`. The fallback `Cross(Cross(LOS, RelVel), LOS)` is correct. But the primary `Normalize(LOS_Delta)` flips direction on sign changes and is noisy for small LOS_Delta just above the `1e-10` threshold.

### 18. `RejectedAccel` direction survives, magnitude thrown away  **WHAM-confirmed**
`Program.cs:473–476` carefully computes `RejectedAccel = sqrt(MissileAccel² - |Lateral|²)` and adds `LOS_New * RejectedAccel`, then `Normalize`s. Only the direction survives — a heuristic for steering direction, not a budget actually honored.

WHAM keeps the magnitude all the way through (`WHAM.cs:2898–2909`):
```csharp
Vector3D lateralAcceleration = GetLatax(...);                                       // m/s²
double missileAccelSq = missileAcceleration * missileAcceleration;
double diff = missileAccelSq - Math.Min(missileAccelSq, lateralAcceleration.LengthSquared()); // guard neg
return lateralAcceleration + Math.Sqrt(diff) * missileToTargetNorm;                 // m/s², NOT normalized
```
Then the caller multiplies by `missileAcceleration` at the end (`WHAM.cs:1891` / `1925`) to produce an actual thrust command in Newton/mass terms. The pointing vector is only normalized at the very last step in `Update` (`WHAM.cs:2865`), *after* gravity compensation has applied its magnitude-preserving math. Current code destroys magnitude twice (normalize in guidance, then normalize again in thrust alignment) so the oversteer/Pythagorean math produces no actual physical result — only a direction.

## Stale state / resource waste

### 19. `startingDistance` never updated in anti-air mode
`Program.cs:177`. Used in the commented terminal phase and by `ApplyGyroOverride` (`distanceToTarget > startingDistance * 0.33`).

### 20. Settings set every frame
`radar.Enabled/Shoot/SyncAzimuth/SyncElevation/SyncEnableIdleRotation` at `Program.cs:319–325`, `_remoteControl.DampenersOverride = false` at `Program.cs:506`, `_remoteControl.IsMainCockpit = true` at `Program.cs:370`. All should be set once at init.

### 21. Anti-air CustomData re-parse per tick
`Program.cs:271–290`. 60 Hz string allocation + split → GC pressure.

### 22. `CheckForGPSAndStart` keeps running every tick before start
Every pre-start tick re-does the full lookup, parse, and `StringBuilder` build. The bomb-mode block at `Program.cs:118–135` parses `Cached:` into `gpsData2` and throws it away — pure dead code.

## Dead code / unused

- `gpsData2` (Program.cs:126), `predictedMissilePos` (306) — computed, never used.
- `closestThruster` (MissileState + Initialization) — referenced but never assigned.
- `_previousTargetVelocity`, `_hasPreviousTargetVelocity`, `updatesPerSecond`, `soundblock`, `thrustOverride` — declared, never read.
- `Initialize()` — empty.
- `_isTerminalPhase` branch is unreachable (enabling predicate is commented).

## API correctness

- `block.Mass` is valid on `IMyCubeBlock.Mass`, but excludes inventory and construction inventory. Prefer `_remoteControl.CalculateShipMass().PhysicalMass` (WHAM-confirmed, `WHAM.cs:1799`).
- `TryParseGPS` — `parts.Length < 5` is loose. A valid SE GPS splits to 7 parts (`GPS:Name:X:Y:Z:#color:`). You read [2]/[3]/[4] correctly, but malformed 5-part strings with numerics at those slots would silently parse.
- `radar.Shoot = true` every frame plus `ShootOnce()` in the cooldown branch is a dubious interaction with the turret's own fire control.

## SE-specific gotchas confirmed by the reference

### Always use `SafeNormalize`, not `Vector3D.Normalize`
WHAM calls `VectorMath.SafeNormalize` 11 times across the guidance and control paths — never raw `Vector3D.Normalize`. The current missile uses `Vector3D.Normalize` everywhere, including on vectors that can be zero (`LOS_Delta` near perfect alignment, `gravity` in space, `LateralAccelerationComponent` at zero-rate moments). `Vector3D.Normalize(Zero)` does not give a clean zero back in SE's VRageMath — it produces `NaN` components that propagate downstream silently. WHAM's helper (`Classes/VectorMath.cs:6–15`):
```csharp
public static Vector3D SafeNormalize(Vector3D a)
{
    if (Vector3D.IsZero(a))  return Vector3D.Zero;
    if (Vector3D.IsUnit(ref a)) return a;
    return Vector3D.Normalize(a);
}
```
Every `Normalize` in the current `Program.cs`, `Guidance.cs`, `Waypoints.cs` should go through this. At least: `LOS_Old`, `LOS_New`, `Normalize(LateralAccelerationComponent)`, `Normalize(vector_to_target)`, `Normalize(gravityVector)` in `AddLoftedTrajectoryWaypoints`.

### Gyro convention: the axis convention is `pitch = −X, yaw = −Y, roll = −Z`
WHAM documents this at `WHAM.cs:4240–4244`. The current `GyroTurn6` uses `GetAzimuthAndElevation` and then does `GYRO.Pitch = -TRANS_VECT.X`, `GYRO.Yaw = -TRANS_VECT.Y`, `GYRO.Roll = -TRANS_VECT.Z` — signs are actually right. But the *path* to get there (RC-frame azimuth/elevation → transform to world via `CreateWorld` → transpose to gyro local) is fragile. Whip's two-step (world → ship → gyro) with `MatrixD.Transpose` avoids the intermediate `CreateWorld` + quaternion inversion entirely.

### GPS parse gotcha
SE GPS format is `GPS:<name>:<X>:<Y>:<Z>:<color>:` where the color token is optional but the trailing `:` is not always present. `string.Split(':')` produces 6 or 7 tokens, and the `<name>` field can contain spaces but **cannot contain `:`** — so the position is always `[2], [3], [4]`. The current `TryParseGPS` is actually correct on indices, but `parts.Length < 5` permits 5-token strings that aren't valid GPS. Use `< 6`, and consider `double.TryParse` with `CultureInfo.InvariantCulture` — SE writes GPS in invariant culture regardless of the client locale, but default `TryParse` uses the current culture, which breaks on German/French clients where the decimal separator is `,`.

### Mass / thrust / acceleration must be re-sampled
WHAM recomputes mass, thrust, and `missileAcceleration` every `Navigation()` call (`WHAM.cs:1797–1800`). The current code samples once at tick 20 and never again. In SE, mass changes as hydrogen tanks drain, as warheads detach, as subgrids separate at launch. Thrust changes with altitude for atmo/ion. The oversteer clamp you rely on to keep PN from saturating uses `MissileAccel` — if it's wrong, the clamp is wrong.

### Target velocity must be finite-differenced with a remembered value
WHAM's `GuidanceBase.Update` (`WHAM.cs:2852–2857`) computes target acceleration as `(targetVelocity - _lastVelocity) * UpdatesPerSecond`, and nulls `_lastVelocity` on lock reset (`ClearAcceleration()`). The current code's `targetvelocity` field sticks on last radar value after lock loss (finding #5) and `_previousTargetPoS` is only updated at the end of each tick — if `Cached:` GPS is re-parsed mid-tick into `targetPosition`, the finite-difference computed at `Program.cs:415–417` against `_previousTargetPoS` is measuring against a mix of old radar position and new GPS position. Pick one source per tick and stick with it.
