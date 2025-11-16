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
        double _navConstant = 5.0;
        bool isTopdown = false;
        List<Vector3D> _waypoints = new List<Vector3D>();
        List<IMyGyro> _gyros = new List<IMyGyro>();
        List<IMyThrust> _thrusters = new List<IMyThrust>();
        IMyRemoteControl _remoteControl;
        IMyTextSurface lcdMain;
        List<IMyWarhead> _warheads = new List<IMyWarhead>();
        IMyShipMergeBlock _mergeBlock;
        IMySensorBlock _sensor;
        int cooldown = 0;
        bool _isStarted = false;
        bool _isInitialized = false;
        bool _antiairmode = false;
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
                            lcdMain = GridTerminalSystem.GetBlockWithName("Holo LCD") as IMyTextSurface;
                        }

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
                    if (_ticks > 15)
                    {
                        GridTerminalSystem.GetBlocksOfType(_gyros);
                        Vector3D vector_to_target = targetPosition - _remoteControl.GetPosition();
                        Vector3D forward_direction = Vector3D.Normalize(vector_to_target);
                        Vector3D up_vector = -_remoteControl.GetTotalGravity();
                        double uplift_strength = 0.6; // This is your tuning knob! Start around 0.5-1.0

                        // Add the vectors
                        Vector3D combined_direction = forward_direction + (up_vector * uplift_strength);
                        ApplyGyroOverride(combined_direction, _gyros, _remoteControl.WorldMatrix, 0, 0);
                        GridTerminalSystem.GetBlocks(blocks);

                        foreach (var block in blocks)
                        {
                            var functional = block as IMyFunctionalBlock;
                            if (functional != null)
                            {
                                functional.Enabled = true;
                            }

                            // Adds Mass
                            MissileMass += block.Mass;
                        }
                        foreach (var thruster in _thrusters)
                        {
                            //Adds Force Component (Forwards Only)
                            double ThisThrusterThrust = thruster.MaxThrust;
                            MissileThrust += ThisThrusterThrust;
                        }
                    }


                    return;
                }

                if (_antiairmode && armtype != "bomb")
                {

                    string customData = programmableBlock.CustomData;
                    string[] lines = customData.Split(new[] { '\n', '\r' }, StringSplitOptions.RemoveEmptyEntries);
                    StringBuilder newCustomData = new StringBuilder();

                    foreach (string line in lines)
                    {
                        if (line.StartsWith("Cached" + ":"))
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
                        }
                    }
                }
                if (_warheads.Count == 0)
                {
                    GridTerminalSystem.GetBlocksOfType(_warheads);
                    if (_warheads.Count == 0)
                    {
                        Echo("[ERROR] No warheads found!");
                        return;
                    }
                }

                _sensor = GridTerminalSystem.GetBlockWithName("Sensor") as IMySensorBlock;
                _sensor.Enabled = true;
                if (armtype != "bomb")
                {
                    radar = GridTerminalSystem.GetBlockWithName("Radar") as IMyLargeGatlingTurret;
                }

                if (_gyros.Count == 0)
                {
                    GridTerminalSystem.GetBlocksOfType(_gyros);
                }

                Vector3D currentPos = _remoteControl.GetPosition();
                Vector3D currentVelocity = _remoteControl.GetShipVelocities().LinearVelocity;

                Vector3D predictedMissilePos = currentPos + currentVelocity * tickTime;
                double distanceToWaypoint = Vector3D.Distance(currentPos, _waypoints[_currentWaypointIndex]);

                if (_currentWaypointIndex < _waypoints.Count - 1 && distanceToWaypoint <= WAYPOINT_THRESHOLD)
                {
                    _currentWaypointIndex++;
                }

                Vector3D _destination = _waypoints[Math.Min(_currentWaypointIndex, _waypoints.Count - 1)];
                double distanceToTarget = Vector3D.Distance(currentPos, _destination);

                if (armtype != "bomb")
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

                //Bitch Guidance
                Vector3D targetPositionchanged = targetPosition + targetvelocity * 0.016;
                Vector3D targetdirection = targetPositionchanged - currentPos;
                Vector3D instantlosrate;
                Vector3D forwardVector = currentVelocity.Normalized();
                Vector3D normalizedTargetDirection = targetdirection.Normalized();
                double dotProduct = Vector3D.Dot(forwardVector, normalizedTargetDirection);

                double denominator = targetdirection.LengthSquared();

                instantlosrate = Vector3D.Cross(targetdirection, (currentVelocity - targetvelocity)) / targetdirection.LengthSquared();
                double roll = 0;
                MatrixD worldMatrix = _remoteControl.WorldMatrix;
                Vector3D upVector = worldMatrix.Up;
                Vector3D leftVector = worldMatrix.Left;
                Vector3D gravity = _remoteControl.GetNaturalGravity();
                bool inGravity = gravity.LengthSquared() > 0;
                Vector3D gravityDirection = inGravity ? Vector3D.Normalize(gravity) : Vector3D.Zero;
                double Global_Timestep = 0.016;
                Vector3D MissilePosition = _remoteControl.CubeGrid.WorldVolume.Center;
                Vector3D MissilePositionPrev = _oldmissilePos;
                Vector3D MissileVelocity = (MissilePosition - MissilePositionPrev) / Global_Timestep;

                Vector3D TargetPosition = targetPositionchanged;
                Vector3D TargetPositionPrev = _previousTargetPoS;
                Vector3D TargetVelocity = (TargetPosition - _previousTargetPoS) / Global_Timestep;
                Vector3D LOS_Delta;
                Vector3D LOS_Old = Vector3D.Normalize(TargetPositionPrev - MissilePositionPrev);
                Vector3D LOS_New = Vector3D.Normalize(TargetPosition - MissilePosition);
                Vector3D Rel_Vel = Vector3D.Normalize(TargetVelocity - MissileVelocity);
                double LOS_Rate;
                if (LOS_Old.Length() == 0)
                { LOS_Delta = new Vector3D(0, 0, 0); LOS_Rate = 0.0; }
                else
                { LOS_Delta = LOS_New - LOS_Old; LOS_Rate = LOS_Delta.Length() / Global_Timestep; }
                double Vclosing = (TargetVelocity - MissileVelocity).Length();

                Vector3D LateralDirection = Vector3D.Normalize(Vector3D.Cross(Vector3D.Cross(Rel_Vel, LOS_New), Rel_Vel));
                Vector3D LateralAccelerationComponent = LateralDirection * 5 * LOS_Rate * Vclosing + LOS_Delta * 9.8 * (0.5 * 5);
                //Calculates Remaining Force Component And Adds Along LOS

                MissileAccel = MissileThrust / MissileMass;
                //If Impossible Solution (ie maxes turn rate) Use Drift Cancelling For Minimum T
                double OversteerReqt = (LateralAccelerationComponent).Length() / MissileAccel;
                if (OversteerReqt > 0.98)
                {
                    LateralAccelerationComponent = MissileAccel * Vector3D.Normalize(LateralAccelerationComponent + (OversteerReqt * Vector3D.Normalize(-MissileVelocity)) * 40);
                }

                double RejectedAccel = Math.Sqrt(MissileAccel * MissileAccel - LateralAccelerationComponent.LengthSquared()); //Accel has to be determined whichever way you slice it
                if (double.IsNaN(RejectedAccel)) { RejectedAccel = 0; }
                LateralAccelerationComponent = LateralAccelerationComponent + LOS_New * RejectedAccel;
                Vector3D desiredAcceleration = Vector3D.Normalize(LateralAccelerationComponent - gravity);
                double Yaw = 0; double Pitch = 0;
                foreach (var gyro in _gyros)
                {
                    GyroTurn6(desiredAcceleration, 18, 0.3, _thrusters[0], gyro as IMyGyro, PREV_Yaw, PREV_Pitch, out Pitch, out Yaw);
                }
                PREV_Yaw = Yaw;
                PREV_Pitch = Pitch;


                //Vector3D desiredAcceleration = 3*Vector3D.Cross(currentVelocity, instantlosrate);

                // --- Use the results ---
                // Apply thrust based on 'thrustOverride'
                // Maybe display 'currentMaxSpeed' or use it for other decisions.
                _remoteControl.DampenersOverride = false;
                foreach (var thruster in _thrusters)
                {

                    thruster.ThrustOverridePercentage = 1f;

                }
                // Compute the rotation vector
                // Proceed with rotation and gyro control using desiredAcceleration as calculated
                //ApplyGyroOverride(desiredAcceleration, _gyros, _remoteControl.WorldMatrix, distanceToTarget, startingDistance);
                double speed = currentVelocity.Length();


                _previousTargetPoS = targetPosition;
                _oldmissilePos = _remoteControl.GetPosition();
                if (lcdMain != null)
                {
                    DisplayOnLCD(lcdMain, distanceToTarget, startingDistance, speed, _ticks, desiredAcceleration, desiredAcceleration);
                }
            }
        }

        void GyroTurn6(Vector3D TARGETVECTOR, double GAIN, double DAMPINGGAIN, IMyTerminalBlock REF, IMyGyro GYRO, double YawPrev, double PitchPrev, out double NewPitch, out double NewYaw)
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
            ShipForwardAzimuth = ShipForwardAzimuth + DAMPINGGAIN * ((ShipForwardAzimuth - YawPrev) / 0.016);
            ShipForwardElevation = ShipForwardElevation + DAMPINGGAIN * ((ShipForwardElevation - PitchPrev) / 0.016);

            //Does Some Rotations To Provide For any Gyro-Orientation
            var REF_Matrix = MatrixD.CreateWorld(REF.GetPosition(), (Vector3)ShipForward, (Vector3)ShipUp).GetOrientation();
            var Vector = Vector3.Transform((new Vector3D(ShipForwardElevation, ShipForwardAzimuth, 0)), REF_Matrix); //Converts To World
            var TRANS_VECT = Vector3.Transform(Vector, Matrix.Transpose(GYRO.WorldMatrix.GetOrientation()));  //Converts To Gyro Local

            //Logic Checks for NaN's
            if (double.IsNaN(TRANS_VECT.X) || double.IsNaN(TRANS_VECT.Y) || double.IsNaN(TRANS_VECT.Z))
            { return; }

            //Applies To Scenario
            GYRO.Pitch = (float)MathHelper.Clamp((-TRANS_VECT.X) * GAIN, -1000, 1000);
            GYRO.Yaw = (float)MathHelper.Clamp(((-TRANS_VECT.Y)) * GAIN, -1000, 1000);
            GYRO.Roll = (float)MathHelper.Clamp(((-TRANS_VECT.Z)) * GAIN, -1000, 1000);
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
        private const int GridSize = 15; // Must be odd for a clear center. 7x7 grid.
        private const char EmptyChar = '.';
        private const char CenterChar = 'O';
        private const char TargetChar = '*';

        /// <summary>
        /// Helper method to generate an ASCII art visualization for a single 2D plane.
        /// (This method remains unchanged from your original provided code)
        /// </summary>
        private static string GeneratePlaneVisualization(
            double val1, char val1AxisLabelPositive, char val1AxisLabelNegative,
            double val2, char val2AxisLabelPositive, char val2AxisLabelNegative,
            string planeTitle,
            char val1DimChar, char val2DimChar)
        {
            StringBuilder sbPlane = new StringBuilder();
            char[,] grid = new char[GridSize, GridSize];
            int center = GridSize / 2;

            // Initialize grid
            for (int r = 0; r < GridSize; r++)
            {
                for (int c = 0; c < GridSize; c++)
                {
                    grid[r, c] = EmptyChar;
                }
            }

            // Mark center
            grid[center, center] = CenterChar;

            // Calculate target cell coordinates
            // val1 maps to columns (horizontal), val2 maps to rows (vertical)
            // For display, +val1 is right, +val2 is up (which means smaller row index)
            int targetCol = center + (int)Math.Round(val1);
            int targetRow = center - (int)Math.Round(val2); // Invert val2 for typical screen Y-axis (up is smaller index)

            // Clamp target coordinates to grid boundaries
            targetCol = Math.Max(0, Math.Min(GridSize - 1, targetCol));
            targetRow = Math.Max(0, Math.Min(GridSize - 1, targetRow));

            // Mark target if it's not the center
            if (targetRow != center || targetCol != center)
            {
                grid[targetRow, targetCol] = TargetChar;
            }

            // Build the string representation of the grid
            sbPlane.AppendLine(planeTitle);

            // Top axis label (for val1)
            sbPlane.Append("  "); // Indent for vertical axis labels
            for (int c = 0; c < GridSize; c++)
            {
                if (c == 0) sbPlane.Append(val1AxisLabelNegative);
                else if (c == GridSize - 1) sbPlane.Append(val1AxisLabelPositive);
                else if (c == center) sbPlane.Append(val1DimChar);
                else sbPlane.Append(" ");
            }
            sbPlane.AppendLine();

            // Grid rows with side axis labels (for val2)
            for (int r = 0; r < GridSize; r++)
            {
                // Side axis label
                if (r == 0) sbPlane.Append(val2AxisLabelPositive).Append(" ");
                else if (r == GridSize - 1) sbPlane.Append(val2AxisLabelNegative).Append(" ");
                else if (r == center) sbPlane.Append(val2DimChar).Append(" ");
                else sbPlane.Append("  ");

                // Grid content for the current row
                for (int c = 0; c < GridSize; c++)
                {
                    sbPlane.Append(grid[r, c]);
                }
                sbPlane.AppendLine();
            }
            return sbPlane.ToString();
        }

        /// <summary>
        /// Generates a simplified ASCII visualization for desired nose movement (Pitch and Yaw).
        /// </summary>
        /// <param name="desiredRotation">The Vector3D to visualize.
        /// Assumes Y component is Pitch and Z component is Yaw.</param>
        /// <returns>A string containing the ASCII visualization for Pitch and Yaw.</returns>
        public static string GetRotationVisualizationText(Vector3D desiredRotation, Vector3D desiredacc)
        {
            StringBuilder result = new StringBuilder();
            double maxDisplayMagnitude = (GridSize - 1) / 2.0;

            // Assuming desiredRotation.X is Roll, .Y is Pitch, .Z is Yaw
            double vRoll = desiredRotation.X;
            double vPitch = desiredRotation.Y;
            double vYaw = desiredRotation.Z;

            // Determine the largest component (Pitch or Yaw) to scale if too large for the grid
            double epsilon = 1e-9;
            double relevantMaxAbs = 0;
            if (Math.Abs(vPitch) > epsilon) relevantMaxAbs = Math.Max(relevantMaxAbs, Math.Abs(vPitch));
            if (Math.Abs(vYaw) > epsilon) relevantMaxAbs = Math.Max(relevantMaxAbs, Math.Abs(vYaw));

            double scaleFactor = 1.0;
            if (relevantMaxAbs > epsilon && relevantMaxAbs > maxDisplayMagnitude)
            {
                scaleFactor = maxDisplayMagnitude / relevantMaxAbs;
            }

            double displayPitch = vPitch * scaleFactor;
            double displayYaw = vYaw * scaleFactor;

            result.AppendLine("Desired Nose Movement (O=Origin, *=Target):");
            result.AppendLine($"Input Rotation: Roll={vRoll:F2}, Pitch={vPitch:F2}, Yaw={vYaw:F2}");
            result.AppendLine($"Input Speed: X={desiredacc.X:F2}, Y={desiredacc.Y:F2}, Z={desiredacc.Z:F2}");
            if (Math.Abs(scaleFactor - 1.0) > epsilon) // Only show scaling info if actually scaled
            {
                result.AppendLine($"Scaled for Display (Factor: {scaleFactor:F2}): dPitch={displayPitch:F2}, dYaw={displayYaw:F2}");
            }
            else
            {
                result.AppendLine($"Display Values: Pitch={displayPitch:F2}, Yaw={displayYaw:F2}");
            }
            result.AppendLine();

            // Visualize Yaw (Left/Right) as val1 (horizontal) and Pitch (Up/Down) as val2 (vertical)
            // Positive Yaw ('R') to the right, Positive Pitch ('U') upwards on the grid.
            result.Append(GeneratePlaneVisualization(
                displayYaw,               // val1 (horizontal: Yaw)
                'R',                      // val1AxisLabelPositive (Yaw Right)
                'L',                      // val1AxisLabelNegative (Yaw Left)
                displayPitch,             // val2 (vertical: Pitch)
                'U',                      // val2AxisLabelPositive (Pitch Up)
                'D',                      // val2AxisLabelNegative (Pitch Down)
                "Nose Aim (U/D: Pitch, L/R: Yaw)",
                'Y',                      // val1DimChar (Yaw axis identifier)
                'P'                       // val2DimChar (Pitch axis identifier)
            ));

            return result.ToString();
        }
        public void DisplayOnLCD(IMyTextSurface lcd, double distanceToTarget, double startingDistance, double currentVelocity, int _ticks, Vector3D gyros, Vector3D desiredacc)
        {
            int barLength = 25;
            float fontSize = 0.5f;
            int quoteScrollingSpeed = 8;
            int scrollTickDivider = 4;
            double timeToTargetSec = currentVelocity > 0 ? distanceToTarget / currentVelocity : double.MaxValue;

            lcd.ContentType = ContentType.TEXT_AND_IMAGE;
            lcd.FontSize = fontSize;
            lcd.Alignment = TextAlignment.CENTER;
            lcd.Font = "Monospace";

            if (timeToTargetSec <= 2.5)
            {
                if (timeToTargetSec > 0.1)
                {
                    int maxExplosionRadius = 20;
                    double explosionIntensity = (2.5 - (timeToTargetSec - 0.1)) / 2.5;
                    int currentRadius = (int)(maxExplosionRadius * explosionIntensity);
                    StringBuilder explosionBuilder = new StringBuilder();
                    string boomText = "\n                           H A V E   A   N I C E   D A Y!";
                    explosionBuilder.Append(boomText);

                    for (int y = -maxExplosionRadius; y <= maxExplosionRadius; y++)
                    {
                        for (int x = -maxExplosionRadius; x <= maxExplosionRadius; x++)
                        {
                            double distanceFromCenter = Math.Sqrt(x * x + y * y);
                            if (distanceFromCenter < currentRadius)
                            {
                                if (distanceFromCenter > currentRadius * 0.7 && distanceFromCenter < currentRadius * 0.9)
                                    explosionBuilder.Append(ColorToChar(255, 69, 0));
                                else if (distanceFromCenter > currentRadius * 0.3 && distanceFromCenter < currentRadius * 0.7)
                                    explosionBuilder.Append(ColorToChar(255, 140, 0));
                                else if (distanceFromCenter <= currentRadius * 0.3)
                                    explosionBuilder.Append(ColorToChar(255, 255, 224));
                                else
                                    explosionBuilder.Append(ColorToChar(255, 255, 0));
                            }
                            else
                            {
                                explosionBuilder.Append(' ');
                            }
                        }
                        explosionBuilder.Append('\n');
                    }
                    lcd.WriteText(explosionBuilder.ToString(), false);
                }
                else
                {
                    lcd.WriteText("", false);
                }
            }
            else
            {
                string headerText = "NYINAH CORP";
                string topLeftLabel = "Velocity: ";
                string topRightLabel = "Time to Impact: ";
                string bottomLeftLabel = "Distance to Target: ";
                string bottomRightLabel = "Total Distance at Start: ";
                string progressLabel = "Progress: ";
                string target = "Target: Refresh - " + cooldown.ToString();
                if (!detectedEntity.IsEmpty())
                {
                    target = "Target: " + detectedEntity.Name;
                }
                string centerMessageLabel = " ";

                // Calculate the progress bar
                double progress = Math.Max(0, Math.Min((startingDistance - distanceToTarget) / startingDistance, 1.0)); // Progress based on distance covered
                int filledLength = (int)(barLength * progress);

                // Animation effect: make the progress bar pulse and change color
                char[] pulseChars = { '=', '#', '*' };
                char pulseChar = pulseChars[_ticks % pulseChars.Length]; // Rotate between '=', '#', and '*' for pulsing effect
                string distanceBar = "|" + new string(pulseChar, filledLength) + new string('-', barLength - filledLength) + "|";

                // Spinner animation
                char[] spinnerChars = { '/', '-', '\\', '|' };
                char spinner = spinnerChars[_ticks % spinnerChars.Length]; // Rotate spinner for animation

                // Scrolling message (scrolls from right to left)
                string message = " Nyinah Corp. wishes you a safe and successful mission! ";
                int scrollIndex = (_ticks / scrollTickDivider) % message.Length; // Slow down the scrolling
                string scrollingMessage = message.Substring(scrollIndex) + message.Substring(0, scrollIndex);

                // Combined quotes into a single string for scrolling
                string combinedQuotes = "|| To confine our attention to terrestrial matters would be to limit the human spirit. - Stephen Hawking " +
                                        "|| That's one small step for man, one giant leap for mankind. - Neil Armstrong " +
                                        "|| The Earth is the cradle of humanity, but mankind cannot stay in the cradle forever. - Konstantin Tsiolkovsky " +
                                        "|| We are all made of star-stuff. - Carl Sagan " +
                                        "|| Space exploration is a force of nature unto itself that no other force in society can rival. - Neil deGrasse Tyson " +
                                        "|| Houston, we have a problem. - Jim Lovell " +
                                        "|| I see Earth! It is so beautiful. - Yuri Gagarin " +
                                        "|| Mars is there, waiting to be reached. - Buzz Aldrin " +
                                        "|| Mystery creates wonder and wonder is the basis of man's desire to understand. - Neil Armstrong " +
                                        "|| The important achievement of Apollo was demonstrating that humanity is not forever chained to this planet. - Neil Armstrong " +
                                        "|| We choose to go to the Moon in this decade and do the other things, not because they are easy, but because they are hard. - John F. Kennedy " +
                                        "|| Across the sea of space, the stars are other suns. - Carl Sagan " +
                                        "|| In the long run, a single-planet species will not survive. - Stephen Hawking " +
                                        "|| Earth is a small town with many neighborhoods in a very big universe. - Ron Garan " +
                                        "|| The Moon is the first milestone on the road to the stars. - Arthur C. Clarke " +
                                        "|| There can be no thought of finishing for aiming for the stars. - Dr. Werner von Braun " +
                                        "|| Space is the breath of art. - Frank Lloyd Wright " +
                                        "|| The sky is not the limit, it is just the beginning. - Lynne Condell " +
                                        "|| I don't know what you could say about a day in which you have seen four beautiful sunsets. - John Glenn " +
                                        "|| Space, the final frontier... - Star Trek " +
                                        "|| In my opinion, the future of space exploration will belong to the private sector. - Stephen Hawking " +
                                        "|| The cosmos is within us. We are made of star-stuff. We are a way for the universe to know itself. - Carl Sagan " +
                                        "|| The nitrogen in our DNA, the calcium in our teeth, the iron in our blood, the carbon in our apple pies were made in the interiors of collapsing stars. We are made of star-stuff. - Carl Sagan " +
                                        "|| The exploration of space will go ahead, whether we join in it or not. - John F. Kennedy " +
                                        "|| Do not look at stars as bright spots only. Try to take in the vastness of the universe. - Maria Mitchell " +
                                        "|| The greatest danger to our future is apathy. - Jane Goodall " +
                                        "|| It's not the plane, it's the pilot. - Top Gun: Maverick " +
                                        "|| Never tell me the odds. - Han Solo, Star Wars " +
                                        "|| Stay on target. - Gold Five, Star Wars " +
                                        "|| To infinity and beyond! - Buzz Lightyear, Toy Story " +
                                        "|| I'm not a great pilot, but I can crash with the best of them. - Sgt. Cole, Halo: Combat Evolved " +
                                        "|| Your journey is long, but the way is prepared. - The Forerunner, Halo " +
                                        "|| A single dream is more powerful than a thousand realities. - J.R.R. Tolkien " +
                                        "|| The stars look very different today. - David Bowie " +
                                        "|| The important thing is not to stop questioning. - Albert Einstein " +
                                        "|| We're all stories in the end. Just make it a good one. - The Doctor, Doctor Who " +
                                        "|| There's a starman waiting in the sky. - David Bowie " +
                                        "|| I find your lack of faith disturbing. - Darth Vader, Star Wars " +
                                        "|| We're going where no one has gone before. - Star Trek " +
                                        "|| I don't believe in the no-win scenario. - Captain Kirk, Star Trek " +
                                        "|| If you want to make an apple pie from scratch, you must first invent the universe. - Carl Sagan " +
                                        "|| Let's fly! - Top Gun: Maverick " +
                                        "|| You are here to make a difference. - Unknown " +
                                        "|| Space... is big. Really big. You just won’t believe how vastly, hugely, mind-bogglingly big it is. - Douglas Adams, The Hitchhiker’s Guide to the Galaxy ";

                int quoteScrollIndex = (_ticks / quoteScrollingSpeed) % combinedQuotes.Length; // Slow scroll even further
                string scrollingQuote = combinedQuotes.Substring(quoteScrollIndex) + combinedQuotes.Substring(0, quoteScrollIndex);


                // Compose the text with decorative elements
                string header = $"+====== {headerText} {spinner} ======+";
                string footer = "+======================================================+";
                string topSeparator = "------------------------------------------------------";
                string bottomSeparator = "------------------------------------------------------";

                // Center and space the top texts
                string topLeftText = $"  {topLeftLabel}{currentVelocity.ToString("0.0")} m/s".PadRight(20);
                string topRightText = $"{topRightLabel}{timeToTargetSec.ToString("0.0")} sec".PadLeft(25) + "  ";

                // Center and space the bottom texts
                string bottomLeftText = $"  {bottomLeftLabel}{distanceToTarget.ToString("0.0")} m".PadRight(10) + "\n" +
                                        $"  {target}X{desiredacc.X.ToString("0.00")}Y{desiredacc.Y.ToString("0.00")}Z{desiredacc.Z.ToString("0.00")}".PadRight(10) + "\n" +
                                        $"  {bottomRightLabel}{startingDistance.ToString("0.0")} m".PadRight(10) + "";
                string progressText = $"  {progressLabel}{distanceBar}".PadRight(25);
                string centerMessage = $"  {scrollingMessage} ".PadRight(25);
                string quoteMessage = $"  {scrollingQuote}".PadRight(25);

                // Render the overlapping X's in ASCII

                // ... [Your existing code to create the final output string] ...
                string footerWithLogo = $"+======================================================+";

                // Replace asciiArrow with asciiBoxCrosshair and asciiCircle in the final output
                string output = $"{header}\n{centerMessage}\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n{topSeparator}\n{topLeftText}{topRightText}\n{bottomSeparator}\n{bottomLeftText}\n{progressText}\n{quoteMessage}\n{footerWithLogo}";

                // Display the text on the LCD
                lcd.WriteText(output, false);
            }
        }

        private static char ColorToChar(int r, int g, int b)
        {
            const double BIT_SPACING = 255.0 / 7.0;
            return (char)(0xe100 + ((int)Math.Round(r / BIT_SPACING) << 6) + ((int)Math.Round(g / BIT_SPACING) << 3) + (int)Math.Round(b / BIT_SPACING));
        }

    }
}
