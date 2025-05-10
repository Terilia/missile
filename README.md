# Space Engineers ProNav Missile Guidance Script
This script has been heavily relying on Whiplash141' code and is an adaptiation of it, that won't require active raycast lock. 

This script provides guidance logic for missiles launched from merge blocks in Space Engineers. It uses Proportional Navigation (ProNav) and supports lofted trajectories, target updates, and warhead detonation. It appears designed to work in conjunction with a main control script (like JetOS) that handles launch commands and target data transfer.

## Features

* **Proportional Navigation (ProNav):** Implements ProNav guidance to intercept targets. The navigation constant (`_navConstant`) can be adjusted (default 5.0, higher for bombs).
* **Waypoint Navigation:** Can follow a series of waypoints before proceeding to the final target.
* **Lofted Trajectory (Topdown Attack):** If `isTopdown` is true (set via Custom Data from a controlling script), the missile will first fly towards a calculated high-altitude point before descending onto the target.
* **Target Source:** Reads initial target GPS coordinates from the *launching* Programmable Block's Custom Data, specifically from a line matching its bay number (e.g., `1:GPS:Target:...`). It expects a controlling script to populate this data just before launch.
* **Air-to-Air Mode:** If `_antiairmode` is true (set via Custom Data), it continuously updates its final waypoint based on `Cached:GPS:` data in the controlling Programmable Block's Custom Data, presumably for tracking moving targets. Uses an onboard radar (`IMyLargeGatlingTurret` named "Radar") to detect targets and potentially refine aim.
* **Guidance Calculation:** Uses a `ProNavGuidance` class which calculates desired acceleration vectors considering target position, velocity (assumed zero for initial GPS), missile position/velocity, and gravity.
* **Thrust Control:** Manages thruster override based on alignment with the target and closing speed to optimize speed and maneuverability. Reduces thrust if speed exceeds dynamic limits based on alignment, increases thrust if below minimum speeds.
* **Gyro Control:** Uses onboard gyroscopes (`IMyGyro`) to steer the missile based on the calculated guidance vector.
* **Detonation Logic:**
    * Detonates onboard warheads (`IMyWarhead`) when within `DETONATION_DISTANCE` (8.0m) of the final target.
    * Can perform a raycast check using a camera ("ProxCam") and detonate if an object is detected within 3m (likely proximity fuse).
    * Supports manual detonation via the "detonate" argument.
* **LCD Display:** Outputs status information (distance, velocity, time-to-impact, progress bar, target info) to an LCD panel named "Holo LCD".
* **Bomb Mode:** If `armtype` is set to "bomb", it adjusts the navigation constant and includes logic (`EvaluateBombingSuccess`, `SimulateBombDrop`) to predict impact points and potentially record accuracy data back to the launching Programmable Block's Custom Data (`DataSlotX:`).

## Setup

1.  **Missile Design:** Build your missile as a separate grid with:
    * A Programmable Block containing this script.
    * An `IMyRemoteControl` named "Remote Control Missile".
    * Thrusters (including "Sci-Fi" named ones for primary thrust).
    * Gyroscopes (`IMyGyro`).
    * Warheads (`IMyWarhead`).
    * A Merge Block (`IMyShipMergeBlock`) facing outwards, named according to the bay it will occupy (e.g., "Bay 1", "Bay 2"). The script automatically finds the closest merge block with "Bay" in its name to determine its bay number.
    * (Optional) An LCD Panel named "Holo LCD" for status display.
    * (Optional) A Camera named "ProxCam" for proximity detonation.
    * (Optional for ATA) A Gatling Turret named "Radar".
    * (Optional) A Sound Block named "pain" (used for debugging sound?).
2.  **Launcher Setup:** The launching grid (e.g., a jet using JetOS) needs:
    * Merge blocks corresponding to the missile bay names.
    * A Programmable Block (e.g., running JetOS) capable of:
        * Writing the target GPS string to the correct line in its own Custom Data (e.g., `1:GPS:Target:X:Y:Z:#Color:` for Bay 1) *after* triggering the launch.
        * Setting the `Topdown:true/false` and `AntiAir:true/false` flags in its Custom Data.
        * Triggering the missile's merge block to disconnect (`Enabled=false` or using an action). This script automatically starts guidance once the merge block disconnects and it reads its target GPS.
3.  **Compile:** Check the script in the missile's Programmable Block.

## Usage

1.  **Target Acquisition:** The launching platform's script (e.g., JetOS) needs to acquire a target GPS coordinate.
2.  **Launch:** The launcher script triggers the merge block disconnect for the selected missile(s).
3.  **Data Transfer:** *Immediately after* triggering the launch, the launcher script must write the target GPS data into its own Custom Data in the format `<BayNumber>:GPS:Target:X:Y:Z:#Color:` (e.g., `1:GPS:Target:100:200:300:#FF0000:`). It should also ensure the `Topdown` and `AntiAir` flags are set correctly in its Custom Data.
4.  **Guidance:** Once disconnected, the missile script reads its bay number from the merge block name, finds the corresponding `<BayNumber>:GPS:Target:...` line in the *launcher's* Programmable Block Custom Data, parses the target, and initiates guidance.
5.  **Detonation:** The missile detonates automatically based on proximity or raycast, or can be manually triggered using the `detonate` argument sent to the missile's Programmable Block.

**Note:** This script relies heavily on coordination with a launching script (like JetOS) for target information and launch initiation. Ensure the Custom Data formats and block naming conventions match between the launcher and the missile.

## Detailed Explanations

Here's a breakdown of how some key functionalities work:

* **Initialization & Target Acquisition:**
    * On startup, the script finds the closest `IMyShipMergeBlock` on its own grid with "Bay" in its name. It extracts the number from this name (e.g., "Bay 1" -> `_bayNumber = 1`).
    * It waits until the merge block disconnects (which signifies launch).
    * Once launched (`_isStarted = true`), it reads the Custom Data from the *launching* Programmable Block (identified as "JETOS Programmable Block").
    * It searches for the line corresponding to its bay number (e.g., `1:`) and parses the GPS coordinates that follow to get its initial `targetPosition`.
    * If `_antiairmode` is true, it will *continuously* re-read the `Cached:GPS:` line from the launcher's Custom Data in its main loop, updating its final waypoint to track moving targets designated by the launcher.
* **Proportional Navigation (ProNav) Guidance (`ProNavGuidance`, `CalculateDesiredAcceleration`):**
    * The core idea is to make the missile's rate of turn proportional to the rate of change of the line-of-sight (LOS) to the target.
    * It calculates the vector from the missile to the target (`positionError`).
    * It calculates the relative velocity between the missile and the target (`velocityError`).
    * It calculates a "turn axis" by taking the cross product of the `positionError` and the (smoothed) `velocityError`. This axis is perpendicular to the plane containing the missile, the target, and the relative velocity vector.
    * The main turning command (`proportionalTerm`) is generated by taking the cross product of this `turnAxis` and the direction to the target. The magnitude of this turn is scaled by the `NavConstant` and the magnitude of the relative velocity. This effectively steers the missile to reduce the rotation of the line-of-sight.
    * It adds compensation for gravity (`gravityCompensation`).
    * The final `desiredAcceleration` vector combines this turning command with a component pointing directly towards the target, ensuring the missile closes the distance while correcting its trajectory. The magnitude of the correction is limited by the missile's `maxAcceleration`.
* **Lofted Trajectory (`AddLoftedTrajectoryWaypoints`):**
    * This function is called once at the start if `isTopdown` is true.
    * It takes the missile's current position and the final target position.
    * It calculates the direction vector from the missile to the target.
    * It determines the "Up" direction based on the opposite of the natural gravity vector.
    * It projects the target direction vector onto the horizontal plane (perpendicular to "Up").
    * It calculates an intermediate point along this horizontal direction, typically at a fraction (`fraction`, e.g., 0.5) of the total horizontal distance.
    * It then raises this intermediate point vertically (along the "Up" direction) by a specified `loftHeight`.
    * This calculated high-altitude point is inserted *before* the original target point in the `_waypoints` list. The missile will navigate to the loft point first, then proceed to the final target.
* **Thrust Control Logic:**
    * Calculates the missile's alignment with the target (`velocityAlignmentFactor` - dot product of velocity direction and target direction).
    * Calculates the closing speed towards the target (`closingSpeed`).
    * Defines several speed/alignment thresholds (`ALIGNMENT_THRESHOLD`, `VERY_HIGH_ALIGNMENT_THRESHOLD`, `MIN_SPEED_FOR_THRUST_INCREASE`, `LOWER_SPEED_THRUST_RAMP`).
    * Determines a dynamic maximum speed (`currentMaxSpeed`) based on how good the alignment is. If alignment is very high (above `VERY_HIGH_ALIGNMENT_THRESHOLD`), the max speed increases towards `BOOST_MAX_SPEED`. Otherwise, it stays closer to `BASE_MAX_SPEED`.
    * **Thrust Reduction:** If the missile is generally aligned (`> ALIGNMENT_THRESHOLD`) but its `closingSpeed` exceeds the calculated `currentMaxSpeed` (and it's not the final waypoint), thrust override is set to 0 to prevent overshooting.
    * **Thrust Increase:** If the missile is aligned but `closingSpeed` is below `MIN_SPEED_FOR_THRUST_INCREASE`, the thrust override is increased (potentially interpolated between 0 and 1 based on how far below the threshold it is, down to `LOWER_SPEED_THRUST_RAMP`).
    * **Thrust Cut (Turning):** If the missile is *not* well aligned (`<= ALIGNMENT_THRESHOLD`), thrust override is set to 0 to allow the gyros to turn the missile more effectively without fighting forward thrust.
    * The final calculated `thrustOverride` percentage is applied to all main thrusters.
* **Gyro Control (`ApplyGyroOverride`):**
    * Takes the `desiredAcceleration` vector calculated by the guidance logic.
    * Transforms this *world-space* acceleration vector into the *local coordinate system* of each individual gyroscope.
    * Assigns the X, Y, and Z components of this local vector to the gyro's Pitch, Yaw, and Roll overrides, respectively (potentially with sign changes depending on how the gyro is oriented relative to the missile's axes).
    * Enables gyro override mode.
