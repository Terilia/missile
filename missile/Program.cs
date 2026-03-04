using Sandbox.Game.Entities;
using Sandbox.Game.EntityComponents;
using Sandbox.Game.Lights;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;
using System.Text;
using VRage;
using VRage.Collections;
using VRage.Game;
using VRage.Game.Components;
using VRage.Game.GUI.TextPanel;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.Game.ObjectBuilders.Definitions;
using VRageMath;

namespace IngameScript
{
    partial class Program : MyGridProgram
    {
        // This file contains your actual script.
        //
        // You can either keep all your code here, or you can create separate
        // code files to make your program easier to navigate while coding.
        //
        // Go to:
        // https://github.com/malware-dev/MDK-SE/wiki/Quick-Introduction-to-Space-Engineers-Ingame-Scripts
        //
        // to learn more about ingame scripts.

        // R e a d m e
        // -----------
        // 
        // In this file you can include any instructions or other comments you want to have injected onto the 
        // top of your final script. You can safely delete this file if you do not want any such comments.
        // 
        // This file contains your actual script.
        //
        // You can either keep all your code here, or you can create separate
        // code files to make your program easier to navigate while coding.
        //
        // Go to:
        // https://github.com/malware-dev/MDK-SE/wiki/Quick-Introduction-to-Space-Engineers-Ingame-Scripts
        //
        // to learn more about ingame scripts.

        public Program()
        {
            // The constructor, called only once every session and
            // always before any other method is called. Use it to
            // initialize your script. 
            //     
            // The constructor is optional and can be removed if not
            // needed.
            // 
            // It's recommended to set Runtime.UpdateFrequency 
            // here, which will allow your script to run itself without a 
            // timer block.
            Runtime.UpdateFrequency = UpdateFrequency.Update1;

        }

        public void Save()
        {
            // Called when the program needs to save its state. Use
            // this method to save your state to the Storage field
            // or some other means. 
            // 
            // This method is optional and can be removed if not
            // needed.
        }
        const double WAYPOINT_THRESHOLD = 550.0;
        const double DETONATION_DISTANCE = 8.0;
        const float tickTime = 1f / 60f;
        double _navConstant = 4.0;
        bool isTopdown = false;
        List<Vector3D> _waypoints = new List<Vector3D>();
        List<IMyGyro> _gyros = new List<IMyGyro>();
        List<IMyThrust> _thrusters = new List<IMyThrust>();
        IMyRemoteControl _remoteControl;
        IMyTextPanel lcdMain;
        List<IMyWarhead> _warheads = new List<IMyWarhead>();
        IMyShipMergeBlock _mergeBlock;
        IMySensorBlock _sensor;
        int cooldown = 0;
        bool _isStarted = false;
        bool _isInitialized = false;
        bool _antiairmode = false;
        bool _blocksFullyInitialized = false;
        bool _isTerminalPhase = false;
        Vector3D _previousTargetVelocity = Vector3D.Zero;
        Vector3D _previousTargetPoS = Vector3D.Zero;
        Vector3D targetvelocity = Vector3D.Zero;
        bool _hasPreviousTargetVelocity = false;
        int _ticks = 0;
        int _currentWaypointIndex = 0;
        int _bayNumber;
        bool firstrun = true;
        IMyThrust closestThruster = null;
        MyDetectedEntityInfo detectedEntity;
        double startingDistance = 0;
        IMyProgrammableBlock programmableBlock;
        IMySoundBlock soundblock;
        IMyLargeGatlingTurret radar;
        float thrustOverride = 0f;
        double updatesPerSecond = 60.0; // Adjust if your script runs at a different rate
        string armtype = "missile"; // 0 = fox 1, 1 = fox 3, 2 = bomb
        public Vector3D _oldmissilePos = Vector3D.Zero;
        public double PREV_Yaw = 0; //for PID
        public double PREV_Pitch = 0; //for PID
        /// <summary>
        /// Simulates a bomb drop in Space Engineers with limited steering, gravity from a Remote Control,
        /// optional drag, and a maximum speed clamp.
        /// </summary>
        /// 
        double MissileThrust = 0;
        double MissileAccel = 0;
        double MissileMass = 0;
        List<IMyTerminalBlock> blocks = new List<IMyTerminalBlock>();


        private static double Magnitude(Vector3D vector)
        {
            return Math.Sqrt(MagnitudeSquared(vector));
        }
        public static double MagnitudeSquared(Vector3D vector)
        {
            return vector.X * vector.X + vector.Y * vector.Y + vector.Z * vector.Z;
        }




        void Initialize()
        {

            // Other initialization code...
        }



        Vector3D targetPosition;
        public void Main(string argument, UpdateType updateSource)
        {
            Echo(_ticks.ToString());
            if (argument == "detonate")
            {
                DetonateWarheads();
            }
            if (armtype == "bomb")
            {
                _navConstant = 9.0;
            }
            if (!_isInitialized)
            {
                _ticks++;
                if (_ticks % 5 == 0)
                {
                    _mergeBlock = FindClosestMergeBlock(Me);
                    if (_mergeBlock != null)
                    {
                        _bayNumber = GetBayNumberFromMergeBlock(_mergeBlock);
                        _isInitialized = true;
                        Initialize();
                        Echo($"[INIT] Bay Number: {_bayNumber}");

                    }
                    else
                    {
                        Echo("[INIT] Searching for merge block...");
                    }
                }
                return;
            }

            Echo($"[STATUS] Current Bay Number: {_bayNumber}");

            if (!_isStarted)
            {
                Echo("[STATUS] Checking for GPS and preparing to start...");
                CheckForGPSAndStart();
                if (armtype == "bomb")
                {
                    _remoteControl = GridTerminalSystem.GetBlockWithName("Remote Control Missile") as IMyRemoteControl;
                    programmableBlock = GridTerminalSystem.GetBlockWithName("JETOS Programmable Block") as IMyProgrammableBlock;


                    string customData = programmableBlock.CustomData;
                    string[] lines = customData.Split(new[] { '\n', '\r' }, StringSplitOptions.RemoveEmptyEntries);
                    string gpsData2 = "";
                    foreach (string line2 in lines)
                    {
                        if (line2.StartsWith("Cached:"))
                        {
                            gpsData2 = line2.Substring(line2.IndexOf(':') + 1).Trim();
                        }
                    }

                }
            }
            if (_isStarted)
            {
                _ticks++;


                if (_ticks < 100)
                {

                    if (_ticks < 10 && _ticks > 4)
                    {
                        _remoteControl = GridTerminalSystem.GetBlockWithName("Remote Control Missile") as IMyRemoteControl;
                        if (armtype != "bomb")
                        {
                            lcdMain = GridTerminalSystem.GetBlockWithName("Holo LCD") as IMyTextPanel;
                            if (lcdMain != null)
                                Echo("[LCD] Holo LCD found and initialized");
                            else
                                Echo("[ERROR] Holo LCD not found!");
                        }

                        // Initialize critical blocks once during startup
                        _sensor = GridTerminalSystem.GetBlockWithName("Sensor") as IMySensorBlock;
                        if (_sensor != null) _sensor.Enabled = true;

                        if (armtype != "bomb")
                        {
                            radar = GridTerminalSystem.GetBlockWithName("Radar") as IMyLargeGatlingTurret;
                        }

                        // Initialize warheads
                        GridTerminalSystem.GetBlocksOfType(_warheads);

                        // Initialize gyros
                        GridTerminalSystem.GetBlocksOfType(_gyros);

                        _blocksFullyInitialized = true;

                        //soundblock = GridTerminalSystem.GetBlockWithName("pain") as IMySoundBlock;
                        //soundblock.SelectedSound = "Christ";
                        //soundblock.Play();
                        startingDistance = Vector3D.Distance(_remoteControl.GetPosition(), _waypoints[_waypoints.Count - 1]);
                        string targetName = "Sci-Fi";
                        GridTerminalSystem.GetBlocksOfType(_thrusters);
                        List<IMyThrust> filteredThrusters = new List<IMyThrust>();
                        if (armtype != "bomb")
                        {

                            foreach (var thruster in _thrusters)
                            {
                                if (thruster.CustomName.Contains(targetName))
                                {
                                    filteredThrusters.Add(thruster);

                                }
                            }

                            if (filteredThrusters.Count == 0)
                            {
                                Echo("No thrusters found!");
                                return;
                            }

                            InitializeThrusters();
                        }



                        if (_remoteControl == null)
                        {
                            Echo("[ERROR] Remote Control not found!");
                            throw new Exception("Remote Control not found!");
                        }

                        if (firstrun && isTopdown && !_antiairmode && armtype != "bomb")
                        {
                            AddLoftedTrajectoryWaypoints();
                            firstrun = false;
                            Echo("[WAYPOINTS] Lofted trajectory waypoints added.");
                        }

                    }
                    if (_ticks > 15 && _ticks < 30)
                    {
                        // Initial orientation during early startup
                        Vector3D vector_to_target = targetPosition - _remoteControl.GetPosition();
                        Vector3D forward_direction = Vector3D.Normalize(vector_to_target);
                        Vector3D up_vector = -_remoteControl.GetTotalGravity();
                        double uplift_strength = 0.6; // This is your tuning knob! Start around 0.5-1.0

                        // Add the vectors
                        Vector3D combined_direction = forward_direction + (up_vector * uplift_strength);
                        ApplyGyroOverride(combined_direction, _gyros, _remoteControl.WorldMatrix, 0, 0);

                        // Enable all blocks once during initialization
                        if (_ticks == 16)
                        {
                            GridTerminalSystem.GetBlocks(blocks);
                            foreach (var block in blocks)
                            {
                                var functional = block as IMyFunctionalBlock;
                                if (functional != null)
                                {
                                    functional.Enabled = true;
                                }
                            }
                        }

                        // Calculate mass and thrust (first time)
                        if (_ticks == 20)
                        {
                            MissileMass = 0;
                            MissileThrust = 0;
                            GridTerminalSystem.GetBlocks(blocks);
                            foreach (var block in blocks)
                            {
                                MissileMass += block.Mass;
                            }
                            foreach (var thruster in _thrusters)
                            {
                                MissileThrust += thruster.MaxThrust;
                            }
                        }
                    }


                    return;
                }

                // Mass is calculated once during initialization (tick 20)
                // No need for frequent recalculation

                if (_antiairmode && armtype != "bomb")
                {
                    // Parse CustomData to update target position (runs every tick at 60Hz for real-time tracking)
                    string customData = programmableBlock.CustomData;
                    string[] lines = customData.Split(new[] { '\n', '\r' }, StringSplitOptions.RemoveEmptyEntries);

                    foreach (string line in lines)
                    {
                        if (line.StartsWith("Cached:"))
                        {
                            string gpsData = line.Substring(line.IndexOf(':') + 1).Trim();
                            if (!string.IsNullOrEmpty(gpsData))
                            {
                                if (TryParseGPS(gpsData, out targetPosition))
                                {
                                    _waypoints.Clear();
                                    _waypoints.Add(targetPosition);
                                    _currentWaypointIndex = 0;
                                }
                            }
                            break; // Found the Cached line, no need to continue
                        }
                    }
                }

                // Blocks are already initialized during startup (ticks 5-10)
                // No need to re-fetch every tick

                Vector3D currentPos = _remoteControl.GetPosition();
                Vector3D currentVelocity = _remoteControl.GetShipVelocities().LinearVelocity;

                // FIX Issue 3: Initialize previous positions on first guidance frame to avoid huge velocity spikes
                if (_ticks == 100)
                {
                    _oldmissilePos = currentPos;
                    _previousTargetPoS = targetPosition;
                }

                Vector3D predictedMissilePos = currentPos + currentVelocity * tickTime;
                double distanceToWaypoint = Vector3D.Distance(currentPos, _waypoints[_currentWaypointIndex]);

                if (_currentWaypointIndex < _waypoints.Count - 1 && distanceToWaypoint <= WAYPOINT_THRESHOLD)
                {
                    _currentWaypointIndex++;
                }

                Vector3D _destination = _waypoints[Math.Min(_currentWaypointIndex, _waypoints.Count - 1)];
                double distanceToTarget = Vector3D.Distance(currentPos, _destination);

                if (armtype != "bomb" && radar != null)
                {
                    radar.Enabled = true;
                    radar.Shoot = true;
                    radar.SyncAzimuth();
                    radar.SyncElevation();
                    radar.SyncEnableIdleRotation();
                    radar.TargetEnemies = true;
                    radar.TargetStations = true;

                    detectedEntity = radar.GetTargetedEntity();
                    if (detectedEntity.IsEmpty() && distanceToTarget < 6000 && cooldown <= 0)
                    {
                        radar.ShootOnce();
                        //radar.Enabled = false;
                        cooldown = 150;
                        radar.Enabled = true;
                        radar.ShootOnce();
                        detectedEntity = radar.GetTargetedEntity();
                        radar.Azimuth = 0;
                        radar.Elevation = 0;
                        radar.SyncElevation();
                        radar.SyncAzimuth();

                    }
                    else
                    {
                        if (detectedEntity.IsEmpty())
                        {

                            cooldown -= 1;
                            if (cooldown < 0) { cooldown = 0; }
                        }

                    }
                    if (!detectedEntity.IsEmpty())
                    {
                        _waypoints[_waypoints.Count - 1] = detectedEntity.Position;
                        targetvelocity = detectedEntity.Velocity;
                        targetPosition = detectedEntity.Position;
                    }
                }

                // ===== TERMINAL PHASE DISABLED =====
                // ProNav guidance works all the way to impact - terminal phase was causing
                // 500m misses due to missing gravity compensation and triggering too early
                // if (distanceToTarget < (startingDistance * 0.15) && !_isTerminalPhase)
                // {
                //     _isTerminalPhase = true;
                //     Echo("[TERMINAL] Switching to pure pursuit guidance");
                // }


                _remoteControl.IsMainCockpit = true;
                if (distanceToTarget < 500 && PerformRaycastCheck())
                {
                    DetonateWarheads();
                    return;
                }

                if (distanceToTarget <= DETONATION_DISTANCE)
                {
                    DetonateWarheads();
                    return;
                }

                // ===== GUIDANCE CALCULATION =====
                Vector3D gravity = _remoteControl.GetNaturalGravity();
                MissileAccel = MissileThrust / MissileMass;
                Vector3D desiredAcceleration;
                Vector3D LateralAccelerationComponent;

                if (_isTerminalPhase)
                {
                    // FIX Issue 4: LEAD PURSUIT - Predict where target will be at intercept time
                    Vector3D relativeVelocity = currentVelocity - targetvelocity;
                    double closingSpeed = Math.Max(1.0, relativeVelocity.Length());
                    double timeToIntercept = distanceToTarget / closingSpeed;

                    // Predict target position (use targetvelocity if available, otherwise assume stationary)
                    Vector3D predictedTargetPos = targetPosition + targetvelocity * timeToIntercept;
                    Vector3D directionToTarget = Vector3D.Normalize(predictedTargetPos - currentPos);
                    desiredAcceleration = directionToTarget;
                    LateralAccelerationComponent = desiredAcceleration * MissileAccel;
                    Echo("[TERMINAL] Lead pursuit active");
                }
                else
                {
                    // PROPORTIONAL NAVIGATION: Full ProNav guidance with lead
                    double Global_Timestep = tickTime;
                    Vector3D MissilePosition = _remoteControl.GetPosition();
                    Vector3D MissilePositionPrev = _oldmissilePos;
                    // FIX Issue 2: Use API velocity directly instead of position-derived
                    Vector3D MissileVelocity = currentVelocity;

                    Vector3D TargetPosition = targetPosition;
                    Vector3D TargetPositionPrev = _previousTargetPoS;
                    // FIX Issue 1: Use radar velocity when available, fall back to position-derived
                    Vector3D TargetVelocity = (targetvelocity.LengthSquared() > 0)
                        ? targetvelocity
                        : (TargetPosition - _previousTargetPoS) / Global_Timestep;

                    // LOS vectors (missile to target)
                    Vector3D LOS_Old = Vector3D.Normalize(TargetPositionPrev - MissilePositionPrev);
                    Vector3D LOS_New = Vector3D.Normalize(TargetPosition - MissilePosition);

                    // Relative velocity (positive = closing)
                    Vector3D RelativeVelocity = MissileVelocity - TargetVelocity;

                    // FIX: Closing velocity is dot product with LOS, not total magnitude
                    double Vclosing = Vector3D.Dot(RelativeVelocity, LOS_New);
                    if (Vclosing < 1.0) Vclosing = 1.0; // Minimum to avoid division issues

                    // LOS rate calculation
                    Vector3D LOS_Delta;
                    double LOS_Rate;
                    if (LOS_Old.LengthSquared() < 0.5) // Check for valid previous LOS
                    {
                        LOS_Delta = Vector3D.Zero;
                        LOS_Rate = 0.0;
                    }
                    else
                    {
                        LOS_Delta = LOS_New - LOS_Old;
                        LOS_Rate = LOS_Delta.Length() / Global_Timestep;
                    }

                    // Lateral direction: perpendicular to LOS in the plane of rotation
                    Vector3D LateralDirection;
                    if (LOS_Delta.LengthSquared() > 1e-10)
                    {
                        LateralDirection = Vector3D.Normalize(LOS_Delta);
                    }
                    else
                    {
                        // Fallback: perpendicular to LOS in plane containing relative velocity
                        Vector3D perpendicular = Vector3D.Cross(LOS_New, RelativeVelocity);
                        if (perpendicular.LengthSquared() > 1e-10)
                            LateralDirection = Vector3D.Normalize(Vector3D.Cross(perpendicular, LOS_New));
                        else
                            LateralDirection = Vector3D.Zero;
                    }

                    // Standard Proportional Navigation: a = N * Vc * LOS_rate
                    // No strange augmentation terms
                    LateralAccelerationComponent = LateralDirection * _navConstant * LOS_Rate * Vclosing;
                    //Calculates Remaining Force Component And Adds Along LOS

                    // If lateral acceleration exceeds available thrust, clamp to maximum
                    double OversteerReqt = LateralAccelerationComponent.Length() / MissileAccel;
                    if (OversteerReqt > 0.98)
                    {
                        // Simply clamp to max available acceleration in the commanded direction
                        LateralAccelerationComponent = Vector3D.Normalize(LateralAccelerationComponent) * MissileAccel * 0.98;
                    }

                    double RejectedAccel = Math.Sqrt(MissileAccel * MissileAccel - LateralAccelerationComponent.LengthSquared()); //Accel has to be determined whichever way you slice it
                    if (double.IsNaN(RejectedAccel)) { RejectedAccel = 0; }
                    LateralAccelerationComponent = LateralAccelerationComponent + LOS_New * RejectedAccel;
                    desiredAcceleration = Vector3D.Normalize(LateralAccelerationComponent - gravity);
                }

                // Calculate adaptive gain to reduce oscillation at close range
                double gainMultiplier = Math.Min(1.0, distanceToTarget / 500.0); // Reduces gain below 500m
                // FIX Issue 5: Increased minimum gain from 20% to 60% for better close-range tracking
                double adaptiveGain = 18.0 * Math.Max(0.6, gainMultiplier); // Minimum 60% gain (10.8)

                // Calculate adaptive damping - increases as distance decreases
                double adaptiveDamping = 0.3 * (1.0 + (500.0 / Math.Max(distanceToTarget, 50.0)));

                double Yaw = 0; double Pitch = 0;
                foreach (var gyro in _gyros)
                {
                    GyroTurn6(desiredAcceleration, adaptiveGain, adaptiveDamping, _thrusters[0], gyro as IMyGyro, PREV_Yaw, PREV_Pitch, out Pitch, out Yaw, distanceToTarget);
                }
                PREV_Yaw = Yaw;
                PREV_Pitch = Pitch;


                //Vector3D desiredAcceleration = 3*Vector3D.Cross(currentVelocity, instantlosrate);

                // --- Smart thrust modulation (50-100% based on alignment) ---
                // Get missile forward direction (thrusters point backward)
                Vector3D MissileForwards = _thrusters[0].WorldMatrix.Backward;

                // Calculate how well aligned we are with desired acceleration
                double ThrustPower = Vector_Projection_Scalar(MissileForwards, Vector3D.Normalize(LateralAccelerationComponent));
                ThrustPower = MathHelper.Clamp(ThrustPower, 0.5, 1.0); // 50-100% range

                _remoteControl.DampenersOverride = false;
                foreach (var thruster in _thrusters)
                {
                    // Only update if changed to reduce command overhead
                    float targetOverride = (float)(thruster.MaxThrust * ThrustPower);
                    if (Math.Abs(thruster.ThrustOverride - targetOverride) > 0.01f)
                    {
                        thruster.ThrustOverride = targetOverride;
                    }
                }
                // Compute the rotation vector
                // Proceed with rotation and gyro control using desiredAcceleration as calculated
                //ApplyGyroOverride(desiredAcceleration, _gyros, _remoteControl.WorldMatrix, distanceToTarget, startingDistance);
                double speed = currentVelocity.Length();


                _previousTargetPoS = targetPosition;
                _oldmissilePos = _remoteControl.GetPosition();

                // Update LCD at 20Hz (every 3 ticks) instead of 60Hz for performance
                if (lcdMain != null && _ticks % 3 == 0)
                {
                    DisplayOnLCD(lcdMain, distanceToTarget, startingDistance, speed, _ticks, currentVelocity);
                }
            }
        }

        void GyroTurn6(Vector3D TARGETVECTOR, double GAIN, double DAMPINGGAIN, IMyTerminalBlock REF, IMyGyro GYRO, double YawPrev, double PitchPrev, out double NewPitch, out double NewYaw, double distanceToTarget)
        {
            //Pre Setting Factors
            NewYaw = 0;
            NewPitch = 0;

            //Retrieving Forwards And Up
            Vector3D ShipUp = REF.WorldMatrix.Up;
            Vector3D ShipForward = REF.WorldMatrix.Backward; //Backward for thrusters

            //Create And Use Inverse Quatinion                   
            Quaternion Quat_Two = Quaternion.CreateFromForwardUp(ShipForward, ShipUp);
            var InvQuat = Quaternion.Inverse(Quat_Two);

            Vector3D DirectionVector = TARGETVECTOR; //RealWorld Target Vector
            Vector3D RCReferenceFrameVector = Vector3D.Transform(DirectionVector, InvQuat); //Target Vector In Terms Of RC Block

            //Convert To Local Azimuth And Elevation
            double ShipForwardAzimuth = 0; double ShipForwardElevation = 0;
            Vector3D.GetAzimuthAndElevation(RCReferenceFrameVector, out ShipForwardAzimuth, out ShipForwardElevation);

            //Post Setting Factors
            NewYaw = ShipForwardAzimuth;
            NewPitch = ShipForwardElevation;

            //Applies Some PID Damping
            ShipForwardAzimuth = ShipForwardAzimuth + DAMPINGGAIN * ((ShipForwardAzimuth - YawPrev) / tickTime);
            ShipForwardElevation = ShipForwardElevation + DAMPINGGAIN * ((ShipForwardElevation - PitchPrev) / tickTime);

            //Does Some Rotations To Provide For any Gyro-Orientation
            var REF_Matrix = MatrixD.CreateWorld(REF.GetPosition(), (Vector3)ShipForward, (Vector3)ShipUp).GetOrientation();
            var Vector = Vector3.Transform((new Vector3D(ShipForwardElevation, ShipForwardAzimuth, 0)), REF_Matrix); //Converts To World
            var TRANS_VECT = Vector3.Transform(Vector, Matrix.Transpose(GYRO.WorldMatrix.GetOrientation()));  //Converts To Gyro Local

            //Logic Checks for NaN's
            if (double.IsNaN(TRANS_VECT.X) || double.IsNaN(TRANS_VECT.Y) || double.IsNaN(TRANS_VECT.Z))
            { return; }

            //Applies To Scenario with reasonable clamps to prevent excessive gyro commands
            GYRO.Pitch = (float)MathHelper.Clamp((-TRANS_VECT.X) * GAIN, -500, 500);
            GYRO.Yaw = (float)MathHelper.Clamp(((-TRANS_VECT.Y)) * GAIN, -500, 500);
            GYRO.Roll = (float)MathHelper.Clamp(((-TRANS_VECT.Z)) * GAIN, -500, 500);
            GYRO.GyroOverride = true;
        }




        void CheckForGPSAndStart()
        {
            programmableBlock = GridTerminalSystem.GetBlockWithName("JETOS Programmable Block") as IMyProgrammableBlock;
            if (programmableBlock == null)
            {
                throw new Exception("JETOS Programmable Block not found!");
            }

            string customData = programmableBlock.CustomData;
            string[] lines = customData.Split(new[] { '\n', '\r' }, StringSplitOptions.RemoveEmptyEntries);
            StringBuilder newCustomData = new StringBuilder();
            foreach (string line in lines)
            {
                if (line.StartsWith("Topdown:"))
                {
                    string value = line.Substring(line.IndexOf(':') + 1).Trim();
                    isTopdown = value.Equals("true", StringComparison.OrdinalIgnoreCase);
                }
                if (line.StartsWith("AntiAir:"))
                {
                    string value = line.Substring(line.IndexOf(':') + 1).Trim();
                    _antiairmode = value.Equals("true", StringComparison.OrdinalIgnoreCase);
                }

                if (line.StartsWith(_bayNumber.ToString() + ":"))
                {
                    string gpsData = line.Substring(line.IndexOf(':') + 1).Trim();
                    if (!string.IsNullOrEmpty(gpsData))
                    {
                        if (TryParseGPS(gpsData, out targetPosition))
                        {
                            _waypoints.Clear();
                            _waypoints.Add(targetPosition);
                            _currentWaypointIndex = 0;
                            _isStarted = true;


                            _mergeBlock.Enabled = false;
                            _ticks = 0;
                            newCustomData.AppendLine($"{_bayNumber}:");
                            continue;
                        }
                        else
                        {
                            Echo("Error: Invalid GPS data.");
                        }
                    }
                }
                newCustomData.AppendLine(line);
            }
            if (_antiairmode)
            {
                programmableBlock.CustomData = newCustomData.ToString();
            }

        }

        bool TryParseGPS(string gps, out Vector3D position)
        {
            position = Vector3D.Zero;
            if (string.IsNullOrWhiteSpace(gps)) return false;

            string[] parts = gps.Split(':');
            if (parts.Length < 5) return false;

            double x, y, z;
            if (double.TryParse(parts[2], out x) && double.TryParse(parts[3], out y) && double.TryParse(parts[4], out z))
            {
                position = new Vector3D(x, y, z);
                return true;
            }
            return false;
        }

        bool PerformRaycastCheck()
        {
            if (armtype == "bomb")
            {
                return false;
            }
            IMyCameraBlock camera = GridTerminalSystem.GetBlockWithName("ProxCam") as IMyCameraBlock;
            if (camera != null && camera.CanScan(3.0))
            {
                MyDetectedEntityInfo hitInfo = camera.Raycast(3.0);
                if (hitInfo.HitPosition.HasValue)
                {
                    return true;
                }
            }
            return false;
        }

        void InitializeThrusters()
        {
            foreach (var thruster in _thrusters)
            {

                thruster.ThrustOverridePercentage = 1f;
                thruster.Enabled = true;
            }
            if (closestThruster != null)
            {
                closestThruster.ThrustOverridePercentage = 1f;
                closestThruster.Enabled = true;
            }
            else
            {
                Echo("No functional thruster found.");
            }
        }

        void AddLoftedTrajectoryWaypoints(double fraction = 0.5, double loftHeight = 9000)
        {
            if (_waypoints.Count >= 1 && _remoteControl != null)
            {
                Vector3D currentPosition = _remoteControl.GetPosition();
                Vector3D finalPoint = _waypoints[0];

                // Get the gravity vector at the current position
                Vector3D gravityVector = _remoteControl.GetNaturalGravity();

                // Check if gravity is present
                if (gravityVector.LengthSquared() == 0)
                {
                    Echo("No gravity detected. Cannot calculate lofted trajectory.");
                    return;
                }

                // "Up" direction is opposite to the gravity vector
                Vector3D upDirection = -Vector3D.Normalize(gravityVector);

                // Calculate the direction vector to the target
                Vector3D directionToTarget = finalPoint - currentPosition;

                // Project the direction vector onto the horizontal plane (perpendicular to up direction)
                Vector3D horizontalDirection = VectorMath.Reject(directionToTarget, upDirection);

                double horizontalDistance = horizontalDirection.Length();

                if (horizontalDistance > 0)
                {
                    Vector3D horizontalDirectionNormalized = horizontalDirection / horizontalDistance;

                    // Calculate the waypoint at a fraction of the horizontal distance to the target
                    Vector3D waypoint = currentPosition + horizontalDirectionNormalized * (horizontalDistance * fraction);

                    // Adjust the waypoint's altitude by moving it along the up direction
                    waypoint += upDirection * loftHeight;

                    // Insert the new waypoint at the beginning of the waypoint list
                    _waypoints.Insert(0, waypoint);
                }
                else
                {
                    // If horizontal distance is zero, set the waypoint directly above the current position
                    Vector3D waypoint = currentPosition + upDirection * loftHeight;
                    _waypoints.Insert(0, waypoint);
                }
            }
        }


        void ApplyGyroOverride(Vector3D worldAngularVelocity, List<IMyGyro> gyros, MatrixD shipWorldMatrix, double distanceToTarget, double startingDistance) // ù(ø, ĥ, Ć)
        {
            foreach (var gyro in gyros)
            {
                var localAngularVelocity = Vector3D.TransformNormal(worldAngularVelocity, MatrixD.Transpose(gyro.WorldMatrix));

                float pitch = (float)(localAngularVelocity.Y);
                float yaw = (float)(localAngularVelocity.X);
                float roll = (float)(localAngularVelocity.Z);
                double roll1 = 0;
                MatrixD worldMatrix = _remoteControl.WorldMatrix;
                Vector3D forwardVector = worldMatrix.Forward;
                Vector3D upVector = worldMatrix.Up;
                Vector3D leftVector = worldMatrix.Left;
                Vector3D gravity = _remoteControl.GetNaturalGravity();
                bool inGravity = gravity.LengthSquared() > 0;
                Vector3D gravityDirection = inGravity ? Vector3D.Normalize(gravity) : Vector3D.Zero;

                if (inGravity)
                {
                    roll1 =
                        Math.Atan2(
                            Vector3D.Dot(leftVector, gravityDirection),
                            Vector3D.Dot(upVector, gravityDirection)
                        ) * (180 / Math.PI);

                    // Normalize roll to [0, 360)
                    // 180 = gravity down (flying normal)
                    // 0 = gravity up (flying inverted)
                    if (roll1 < 0)
                        roll1 += 360;
                }

                float rollFloat = 0;
                float upsidedown = 1;
                if (roll1 < 175)
                {
                    rollFloat = -0.1f;
                }
                if (roll1 >= 185)
                {
                    rollFloat = 0.1f;
                }
                if (roll1 <= 90 || roll >= 270)
                {
                    upsidedown = -1;
                }
                gyro.Enabled = true;
                if (distanceToTarget > startingDistance * 0.33)
                {
                    pitch = 1f * upsidedown;
                }
                gyro.Pitch = -pitch;
                gyro.Yaw = roll;
                gyro.Roll = yaw;
                gyro.GyroOverride = true;
            }
        }



        IMyShipMergeBlock FindClosestMergeBlock(IMyTerminalBlock referenceBlock)
        {
            List<IMyShipMergeBlock> mergeBlocks = new List<IMyShipMergeBlock>();
            GridTerminalSystem.GetBlocksOfType(mergeBlocks, block => block.CubeGrid == referenceBlock.CubeGrid);
            IMyShipMergeBlock closestBlock = null;
            double closestDistanceSquared = double.MaxValue;
            Vector3D referencePosition = referenceBlock.GetPosition();

            foreach (var mergeBlock in mergeBlocks)
            {
                if (mergeBlock.CustomName.StartsWith("Bay", StringComparison.OrdinalIgnoreCase))
                {
                    double distanceSquared = Vector3D.DistanceSquared(referencePosition, mergeBlock.GetPosition());
                    if (mergeBlock.IsFunctional && mergeBlock.Enabled && distanceSquared < closestDistanceSquared)
                    {
                        closestDistanceSquared = distanceSquared;
                        closestBlock = mergeBlock;
                    }
                }
            }
            return closestBlock;
        }

        int GetBayNumberFromMergeBlock(IMyShipMergeBlock mergeBlock)
        {
            string name = mergeBlock.CustomName;
            string digits = new string(name.Where(char.IsDigit).ToArray());
            int bayNumber;

            if (int.TryParse(digits, out bayNumber))
            {
                return bayNumber;
            }

            Echo("Error: Bay number not found in merge block name.");
            return -1;
        }

        public static class VectorMath
        {

            public static Vector3D Projection(Vector3D a, Vector3D b)
            {
                if (Vector3D.IsZero(a) || Vector3D.IsZero(b)) return Vector3D.Zero;
                if (Vector3D.IsUnit(ref b)) return a.Dot(b) * b;
                return a.Dot(b) / b.LengthSquared() * b;
            }

            public static Vector3D Reject(Vector3D a, Vector3D b)
            {
                // Rejects vector a onto vector b, returning the component of a perpendicular to b
                return a - Projection(a, b);
            }
        }

        // Vector projection scalar helper (from reference implementation)
        public static double Vector_Projection_Scalar(Vector3D IN, Vector3D Axis_norm)
        {
            double OUT = Vector3D.Dot(IN, Axis_norm);
            if (double.IsNaN(OUT))
            {
                OUT = 0;
            }
            return OUT;
        }

        void DetonateWarheads()
        {
            if (_warheads.Count == 0)
            {
                GridTerminalSystem.GetBlocksOfType(_warheads);
                if (_warheads.Count == 0)
                {
                    Echo("[ERROR] No warheads found!");
                    return;
                }
            }
            foreach (var warhead in _warheads)
            {
                warhead.Detonate();
            }
        }
        public void DisplayOnLCD(IMyTextPanel lcd, double distanceToTarget, double startingDistance, double currentSpeed, int _ticks, Vector3D currentVelocity)
        {
            // Calculate time to impact and progress
            double timeToTargetSec = currentSpeed > 0 ? distanceToTarget / currentSpeed : double.MaxValue;
            double progress = Math.Max(0, Math.Min((startingDistance - distanceToTarget) / startingDistance, 1.0));

            // Setup LCD for text display (camera compatible)
            lcd.ContentType = ContentType.TEXT_AND_IMAGE;
            lcd.Font = "Monospace";
            lcd.FontSize = 0.8f;
            lcd.Alignment = TextAlignment.LEFT;
            lcd.TextPadding = 2f;

            StringBuilder display = new StringBuilder();

            // Header
            display.AppendLine("=====================================");
            display.AppendLine("   NYINAH CORP MISSILE CAM");
            display.AppendLine("=====================================\n");

            // Critical impact warning (last 2.5 seconds)
            if (timeToTargetSec <= 2.5 && timeToTargetSec > 0.1)
            {
                bool flash = (_ticks / 10) % 2 == 0;
                if (flash)
                {
                    display.AppendLine("!!! IMPACT IMMINENT !!!");
                    display.AppendLine($"    {timeToTargetSec:F1} SECONDS");
                    display.AppendLine("!!! IMPACT IMMINENT !!!");
                }
                else
                {
                    display.AppendLine("");
                    display.AppendLine($">>> {timeToTargetSec:F1} SECONDS <<<");
                    display.AppendLine("");
                }
            }
            else
            {
                // Normal telemetry display
                display.AppendLine($"Speed:     {currentSpeed:F0} m/s");
                display.AppendLine($"Distance:  {distanceToTarget:F0} m");

                if (timeToTargetSec < 999)
                    display.AppendLine($"ETA:       {timeToTargetSec:F1} sec");
                else
                    display.AppendLine("ETA:       ---");

                // Target info
                if (!detectedEntity.IsEmpty())
                    display.AppendLine($"Target:    {detectedEntity.Name}");
                else
                    display.AppendLine("Target:    Seeking...");

                display.AppendLine("");

                // Progress bar (25 characters wide)
                int barLength = 25;
                int filled = (int)(barLength * progress);
                string progressBar = "[" + new string('=', filled) + ">" + new string('-', barLength - filled - 1) + "]";
                display.AppendLine($"Progress:  {progressBar}");
                display.AppendLine($"           {(progress * 100):F0}%");

                display.AppendLine("");

                // Status
                if (distanceToTarget < 100)
                    display.AppendLine("Status:    TERMINAL PHASE");
                else if (distanceToTarget < 500)
                    display.AppendLine("Status:    FINAL APPROACH");
                else
                    display.AppendLine("Status:    TRACKING");
            }

            display.AppendLine("\n=====================================");

            lcd.WriteText(display.ToString());
        }

    }
}
