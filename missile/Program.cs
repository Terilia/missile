using Sandbox.Game.EntityComponents;
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
        ProNavGuidance _proNavGuidance;
        BombDragProNavGuidance _bombDragProNavGuidance;
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
        string armtype = "bomb"; // 0 = fox 1, 1 = fox 3, 2 = bomb

        /// <summary>
        /// Simulates a bomb drop in Space Engineers with limited steering, gravity from a Remote Control,
        /// optional drag, and a maximum speed clamp.
        /// </summary>
        Vector3D SimulateBombDrop(
            Vector3D initialPosition,
            Vector3D initialVelocity,
            IMyRemoteControl remoteControl,
            double timeStep,
            double maxTime,
            Vector3D targetPosition,
            double impactThreshold,
            double dragCoefficient
        )
        {
            double steerStrength = 0.2; // Reduced for smoother control
            double maxSteerAngleDeg = 5.0; // Prevent oversteering
            double estimatedGlideDistance = 690.9; // Computed from previous experiments

            Vector3D position = initialPosition;
            Vector3D velocity = initialVelocity;
            Vector3D tailDirection = -Vector3D.Normalize(velocity); // Track "ass-first" orientation

            double time = 0.0;
            double closestDistance = double.MaxValue;
            Vector3D closestPosition = position;

            while (time < maxTime)
            {
                // Gravity
                Vector3D gravity = remoteControl.GetNaturalGravity();

                // Compute speed
                double speed = velocity.Length();
                Vector3D velocityDir = (speed > 1e-6) ? Vector3D.Normalize(velocity) : Vector3D.Zero;

                // Compute target direction
                Vector3D targetDirection = Vector3D.Normalize(targetPosition - position);

                // Use tail direction for steer calculation
                double angleToTarget = CalculateAngleBetween(tailDirection, targetDirection) * (180.0 / Math.PI);
                if (angleToTarget > maxSteerAngleDeg)
                {
                    targetDirection = Vector3D.Lerp(tailDirection, targetDirection, steerStrength * timeStep);
                    targetDirection = Vector3D.Normalize(targetDirection);
                }

                // Update bomb tail orientation smoothly
                tailDirection = Vector3D.Lerp(tailDirection, targetDirection, steerStrength * timeStep);
                tailDirection = Vector3D.Normalize(tailDirection);

                // Orientation-based drag (favor "ass-first" stability)
                double alignmentFactor = Math.Max(0.2, Vector3D.Dot(tailDirection, velocityDir)); // Prevent complete stall
                Vector3D dragAccel = -dragCoefficient * speed * speed * alignmentFactor * velocityDir;

                // Apply forces
                velocity += (gravity + dragAccel) * timeStep;
                velocity = Vector3D.Normalize(velocity) * speed; // Maintain speed but adjust direction

                // Update position
                Vector3D nextPosition = position + velocity * timeStep;

                // Adjust impact radius estimate
                double currentGlideRadius = estimatedGlideDistance * (speed / maxTime);
                if (Vector3D.Distance(nextPosition, initialPosition) > currentGlideRadius)
                {
                    return nextPosition; // Ensure the bomb stays within expected impact zone
                }

                // Check for impact
                double dist = Vector3D.Distance(nextPosition, targetPosition);
                if (dist < impactThreshold)
                {
                    return nextPosition;
                }

                // Track closest approach
                if (dist < closestDistance)
                {
                    closestDistance = dist;
                    closestPosition = nextPosition;
                }

                position = nextPosition;
                time += timeStep;
            }

            return closestPosition;
        }


        void EvaluateBombingSuccess(Vector3D targetPosition)
        {
            Vector3D bombPosition = _remoteControl.GetPosition();
            Vector3D bombVelocity = _remoteControl.GetShipVelocities().LinearVelocity;
            Vector3D gravity = _remoteControl.GetNaturalGravity();
            Vector3D predictedImpact = SimulateBombDrop(bombPosition, bombVelocity, _remoteControl, 0.1, 200, targetPosition, 2.0, 0.001);
            double distanceToTarget = Vector3D.Distance(predictedImpact, targetPosition) / 2;


            programmableBlock = GridTerminalSystem.GetBlockWithName("JETOS Programmable Block") as IMyProgrammableBlock;
            if (programmableBlock == null)
            {
                throw new Exception("JETOS Programmable Block not found!");
            }

            string customData = programmableBlock.CustomData;
            string[] lines = customData.Split(new[] { '\n', '\r' }, StringSplitOptions.RemoveEmptyEntries);
            StringBuilder newCustomData = new StringBuilder();
            for (int i = 0; i < lines.Length; i++)
            {
                string line = lines[i];
                if (line.StartsWith("DataSlot" + _bayNumber.ToString() + ":"))
                {
                    line = "DataSlot" + _bayNumber.ToString() + ":" + distanceToTarget.ToString("0.0");
                }
                newCustomData.AppendLine(line);
            }

            programmableBlock.CustomData = newCustomData.ToString();
        }

        void Initialize()
        {
            
            // Other initialization code...
        }

        abstract class GuidanceBase
        {
            public double DeltaTime { get; private set; }
            public double UpdatesPerSecond { get; private set; }

            public Vector3D? _lastVelocity;

            public GuidanceBase(double updatesPerSecond)
            {
                UpdatesPerSecond = updatesPerSecond;
                DeltaTime = 1.0 / UpdatesPerSecond;
            }

            public void ClearAcceleration()
            {
                _lastVelocity = null;
            }

            public Vector3D Update(
                Vector3D missilePosition,
                Vector3D missileVelocity,
                double missileAcceleration,
                Vector3D targetPosition,
                Vector3D targetVelocity,
                Vector3D targetAcceleration,
                Vector3D naturalGravity) // Pass gravity as an input
            {
                Vector3D targetAccel = Vector3D.Zero;

                if (_lastVelocity.HasValue)
                    targetAccel = (targetVelocity - _lastVelocity.Value) * UpdatesPerSecond;

                _lastVelocity = targetVelocity;

                // Gravity compensation
                Vector3D gravityCompensation = -naturalGravity;

                Vector3D pointingVector = GetPointingVector(
                    missilePosition,
                    missileVelocity,
                    missileAcceleration,
                    targetPosition,
                    targetVelocity,
                    targetAccel);

                // Add gravity compensation
                return VectorMath.SafeNormalize(pointingVector + gravityCompensation);
            }


            public abstract Vector3D GetPointingVector(
                Vector3D missilePosition,
                Vector3D missileVelocity,
                double missileAcceleration,
                Vector3D targetPosition,
                Vector3D targetVelocity,
                Vector3D targetAcceleration);
        }


        abstract class RelNavGuidance : GuidanceBase
        {
            public double NavConstant;
            public double NavAccelConstant;

            public RelNavGuidance(double updatesPerSecond, double navConstant, double navAccelConstant = 0)
                : base(updatesPerSecond)
            {
                NavConstant = navConstant;
                NavAccelConstant = navAccelConstant;
            }

            protected abstract Vector3D GetLatax(
                Vector3D missileToTarget,
                Vector3D missileToTargetNorm,
                Vector3D relativeVelocity,
                Vector3D lateralTargetAcceleration);

            public override Vector3D GetPointingVector(
                Vector3D missilePosition,
                Vector3D missileVelocity,
                double missileAcceleration,
                Vector3D targetPosition,
                Vector3D targetVelocity,
                Vector3D targetAcceleration)
            {
                Vector3D missileToTarget = targetPosition - missilePosition;
                Vector3D missileToTargetNorm = Vector3D.Normalize(missileToTarget);
                Vector3D relativeVelocity = targetVelocity - missileVelocity;
                Vector3D lateralTargetAcceleration = targetAcceleration - Vector3D.Dot(targetAcceleration, missileToTargetNorm) * missileToTargetNorm;

                Vector3D lateralAcceleration = GetLatax(
                    missileToTarget,
                    missileToTargetNorm,
                    relativeVelocity,
                    lateralTargetAcceleration);

                double missileAccelSq = missileAcceleration * missileAcceleration;
                double diff = missileAccelSq - Math.Min(missileAccelSq, lateralAcceleration.LengthSquared());

                // Add gravity compensation
                return lateralAcceleration + Math.Sqrt(diff) * missileToTargetNorm;
            }

        }
        class ProNavGuidance : RelNavGuidance
        {
            public ProNavGuidance(double updatesPerSecond, double navConstant, IMyRemoteControl remoteControl, double navAccelConstant = 0)
                : base(updatesPerSecond, navConstant, navAccelConstant)
            {
                _remoteControl = remoteControl;
            }

            private Vector3D _previousRelativeVelocity = Vector3D.Zero;
            private readonly IMyRemoteControl _remoteControl;
            protected override Vector3D GetLatax(
        Vector3D missileToTarget,
        Vector3D missileToTargetNorm,
        Vector3D relativeVelocity,
        Vector3D lateralTargetAcceleration)
            {
                Vector3D naturalGravity = _remoteControl.GetNaturalGravity();

                double smoothingFactor = 0.8;
                Vector3D smoothedRelativeVelocity = smoothingFactor * relativeVelocity +
                                                    (1 - smoothingFactor) * _previousRelativeVelocity;

                _previousRelativeVelocity = smoothedRelativeVelocity;

                Vector3D omega = Vector3D.Cross(missileToTarget, smoothedRelativeVelocity) /
                                 Math.Max(missileToTarget.LengthSquared(), 1);

                return NavConstant * smoothedRelativeVelocity.Length() * Vector3D.Cross( missileToTargetNorm, omega)
                       + NavAccelConstant * lateralTargetAcceleration
                       - naturalGravity;
            }





        }


        class BombDragProNavGuidance : ProNavGuidance
        {
            // How often to log (in calls to GetPointingVector).
            private readonly IMyRemoteControl _remoteControl;
            // Simple counter to know when it's time to log.
            public int _tickCounter;
            public Vector3D previousForward;
            private bool _initialVelocityLogged = false;
            // A buffer that accumulates the log lines in memory.
            private readonly List<string> _logEntries = new List<string>();

            // Publicly accessible property to retrieve the log from outside the class.
            public IReadOnlyList<string> LogEntries => _logEntries.AsReadOnly();

            public BombDragProNavGuidance(
                double updatesPerSecond,
                double navConstant,
                IMyRemoteControl remoteControl,
                double navAccelConstant = 0
            )
                : base(updatesPerSecond, navConstant, remoteControl, navAccelConstant)
            {
                _remoteControl = remoteControl;
                _tickCounter = 0;
            }
            double CalculateAngleBetween(Vector3D v1, Vector3D v2)
            {
                double dot = Vector3D.Dot(v1, v2);
                double magProduct = v1.Length() * v2.Length();

                if (magProduct < 1e-6) return 0; // Avoid division by zero

                double cosTheta = MathHelper.Clamp(dot / magProduct, -1.0, 1.0); // Clamp to avoid NaN issues
                return Math.Acos(cosTheta) * (180.0 / Math.PI); // Convert to degrees
            }
            /// <summary>
            /// Overridden so that we only return lateral drag guidance,
            /// not forward thrust or direct gravity compensation.
            /// </summary>
            public override Vector3D GetPointingVector(
    Vector3D missilePosition,
    Vector3D missileVelocity,
    double missileAcceleration,
    Vector3D targetPosition,
    Vector3D targetVelocity,
    Vector3D targetAcceleration
)
            {
                Vector3D missileToTarget = targetPosition - missilePosition;
                if (missileToTarget.LengthSquared() < 1e-6)
                    return Vector3D.Zero;

                Vector3D losDir = Vector3D.Normalize(missileToTarget);
                Vector3D relativeVelocity = targetVelocity - missileVelocity;

                // Recompute target acceleration if you like
                Vector3D targetAccel = Vector3D.Zero;
                if (_lastVelocity.HasValue)
                    targetAccel = (targetVelocity - _lastVelocity.Value) * UpdatesPerSecond;
                _lastVelocity = targetVelocity;

                // Lateral portion of that acceleration
                Vector3D lateralTargetAccel
                    = targetAccel - Vector3D.Dot(targetAccel, losDir) * losDir;

                // Our standard lateral PN
                Vector3D lateralAcceleration = GetLatax(
                    missileToTarget,
                    losDir,
                    relativeVelocity,
                    lateralTargetAccel
                );

                // ---------------------------
                // Example of explicit gravity compensation for a bomb
                // If you want partial, multiply by e.g. 0.5 or so
                // If you want "pure ballistic," skip it entirely.
                // NOTE:  _remoteControl.GetNaturalGravity() is downward,
                // so subtracting it = adding an upward acceleration
                // to try to counter gravity
                // ---------------------------
                var gravity = _remoteControl.GetNaturalGravity(); // downward vector
                Vector3D gravityComp = -gravity; // upward

                // Combine them:
                Vector3D totalAcceleration = lateralAcceleration + gravityComp;

                // --- Logging System ---
                if (!_initialVelocityLogged)
                {
                    _initialVelocityLogged = true;
                    double initSpeed = missileVelocity.Length();
                    _logEntries.Add(
                        $"[Tick={_tickCounter}] InitialVelocity={initSpeed:F2} m/s " +
                        $"Vector=({missileVelocity.X:F2}, {missileVelocity.Y:F2}, {missileVelocity.Z:F2})"
                    );
                }

                // Log key values every 60 ticks
                if (_tickCounter % 60 == 0)
                {
                    double dist = missileToTarget.Length();
                    double latAccelMag = lateralAcceleration.Length();
                    double totalAccelMag = totalAcceleration.Length();
                    double missileSpeed = missileVelocity.Length();
                    double gravMag = gravity.Length();
                    double timeElapsed = _tickCounter / UpdatesPerSecond; // Convert to seconds
                    // Compute current steer angle (angle between forward direction & target)
                    double steerAngle = Math.Acos(Vector3D.Dot(_remoteControl.WorldMatrix.Forward, Vector3D.Normalize(missileToTarget))) * (180.0 / Math.PI);

                    // Compute steer strength effectiveness (change in orientation per tick)
                    double steerEffectiveness = CalculateAngleBetween(previousForward, _remoteControl.WorldMatrix.Forward) * (180.0 / Math.PI);

                    _logEntries.Add(
                        $"[Tick={_tickCounter}]" +
                        $"Dist={dist:F2}m  BSpd={missileSpeed:F2}m/s  " +
                        $"LatAccel={latAccelMag:F2}m/s²  TtlAccl={totalAccelMag:F2}m/s²  " +
                        $"GMag={gravMag:F2}m/s²  " +
                        $"BPos=({missilePosition.X:F2}, {missilePosition.Y:F2}, {missilePosition.Z:F2})  " +
                        $"RlVel=({relativeVelocity.X:F2}, {relativeVelocity.Y:F2}, {relativeVelocity.Z:F2})  " +
                        $"SteerAngle={steerAngle:F2}°  SteerEffect={steerEffectiveness:F2}°/tick"
                    );

                    // Update previous forward direction for next tick comparison
                    previousForward = _remoteControl.WorldMatrix.Forward;
                }

                _tickCounter++;
                
                return totalAcceleration;
            }


        }




        Vector3D targetPosition;
        public void Main(string argument, UpdateType updateSource)
        {
            Echo(_ticks.ToString());
            if (argument == "detonate")
            {
                DetonateWarheads();
            }
            if(armtype == "bomb")
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
                    if (TryParseGPS(gpsData2, out targetPosition))
                    {
                        EvaluateBombingSuccess(targetPosition);
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
                        if(armtype != "bomb")
                        {
                            lcdMain = GridTerminalSystem.GetBlockWithName("Holo LCD") as IMyTextSurface;
                        }

                        //soundblock = GridTerminalSystem.GetBlockWithName("pain") as IMySoundBlock;
                        //soundblock.SelectedSound = "Christ";
                        //soundblock.Play();
                        _proNavGuidance = new ProNavGuidance(updatesPerSecond, _navConstant, _remoteControl);
                        _bombDragProNavGuidance = new BombDragProNavGuidance(updatesPerSecond, _navConstant, _remoteControl);
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
                    if (detectedEntity.IsEmpty() && distanceToTarget < 3500 && cooldown <= 0)
                    {
                        radar.ShootOnce();
                        //radar.Enabled = false;
                        cooldown = 150;
                        radar.Enabled = true;
                        radar.ShootOnce();
                        detectedEntity = radar.GetTargetedEntity();

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
                    }
                    Vector3D directionToLeadPoint = VectorMath.SafeNormalize(_destination - currentPos);
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
                double missileAcceleration;
                Vector3D naturalgravity = _remoteControl.GetNaturalGravity();
                Vector3D desiredAcceleration;
                if (armtype == "bomb")
                {
                    // Calculate missile acceleration
                    missileAcceleration = 0;
                    // Update the guidance
                    var logEntries = _bombDragProNavGuidance.LogEntries;
                    int maxDisplay = Math.Min(40, logEntries.Count);
                    string base64Encoded = "";
                    for (int i = 0; i < maxDisplay; i++)
                    {
                        base64Encoded += logEntries[i];
                        
                    }
                    Echo(Convert.ToBase64String(Encoding.UTF8.GetBytes(base64Encoded)));
                    desiredAcceleration = _bombDragProNavGuidance.Update(
                        missilePosition: _remoteControl.GetPosition(),
                        missileVelocity: currentVelocity,
                        missileAcceleration: missileAcceleration,
                        targetPosition: _destination,
                        targetVelocity: Vector3D.Zero, // Use actual target velocity if available
                        targetAcceleration: Vector3D.Zero,
                        naturalGravity: naturalgravity// Use actual target acceleration if available
                    );
                }
                else
                {
                    // Calculate missile acceleration
                    missileAcceleration = CalculateMissileAcceleration();
                    // Update the guidance
                    desiredAcceleration = _proNavGuidance.Update(
                        missilePosition: _remoteControl.GetPosition(),
                        missileVelocity: currentVelocity,
                        missileAcceleration: missileAcceleration,
                        targetPosition: _destination,
                        targetVelocity: Vector3D.Zero, // Use actual target velocity if available
                        targetAcceleration: Vector3D.Zero,
                        naturalGravity: naturalgravity// Use actual target acceleration if available
                    );
                }

                // Compute the rotation vector
                Vector3D rotationVector = ComputeRotationVector(desiredAcceleration, _remoteControl.WorldMatrix);

                ApplyGyroOverride(rotationVector, _gyros, _remoteControl.WorldMatrix);
                double speed = currentVelocity.Length();



                Vector3D toTarget = _destination - currentPos;
                Vector3D toTargetDirection = Vector3D.Normalize(toTarget);

                double alignment = Vector3D.Dot(_remoteControl.WorldMatrix.Forward, toTargetDirection);

                double speedTowardsTarget = Vector3D.Dot(currentVelocity, toTargetDirection);

                // Only apply thrust if we're sufficiently facing the target
                const double alignmentThreshold = 0.95; // Adjust as desired

                if (alignment > alignmentThreshold)
                {
                    if (speedTowardsTarget > 350 && _waypoints.Count > 1)
                    {
                        thrustOverride = 0.0f; // Cut off thrust to prevent overshooting
                    }
                    else if (speedTowardsTarget < 290)
                    {
                        float maxSpeed = 290.0f;
                        float minSpeed = 245.0f;
                        float reductionFactor = (float)(maxSpeed - speedTowardsTarget) / (maxSpeed - minSpeed);
                        thrustOverride = 1f * reductionFactor;
                    }
                }
                else
                {
                    // Not facing the target closely enough, so don't accelerate forward.
                    thrustOverride = 0.0f;
                }

                _remoteControl.DampenersOverride = false;
                foreach (var thruster in _thrusters)
                {

                    thruster.ThrustOverridePercentage = thrustOverride;
                }
                if (lcdMain != null)
                {
                    DisplayOnLCD(lcdMain, distanceToTarget, startingDistance, speed, _ticks);
                }
            }
        }
        double CalculateMissileAcceleration()
        {
            // Sum up the maximum effective thrust of your main thrusters
            if(armtype == "bomb")
            {
                return 0;
            }
            double totalThrust = 0.0;
            foreach (var thruster in _thrusters)
            {
                if (thruster.IsWorking)
                {
                    totalThrust += thruster.MaxEffectiveThrust;
                }
            }

            // Get the mass of the missile
            double missileMass = _remoteControl.CalculateShipMass().PhysicalMass;

            // Calculate acceleration (a = F / m)
            return missileMass > 0 ? totalThrust / missileMass : 0;
        }
        Vector3D ComputeRotationVector(Vector3D desiredAcceleration, MatrixD worldMatrix)
        {
            // Desired direction is the direction of the desired acceleration
            Vector3D desiredDirection = VectorMath.SafeNormalize(desiredAcceleration);

            // Current forward direction of the missile
            Vector3D currentDirection = worldMatrix.Forward;

            // Compute the rotation axis and angle
            Vector3D rotationAxis = Vector3D.Cross(currentDirection, desiredDirection);
            double angle = Math.Acos(MathHelper.Clamp(Vector3D.Dot(currentDirection, desiredDirection), -1, 1));

            // If the angle is very small, no rotation is needed
            if (Math.Abs(angle) < 1e-6)
                return Vector3D.Zero;

            // Normalize the rotation axis
            rotationAxis = VectorMath.SafeNormalize(rotationAxis);

            // Compute the rotation vector (angular velocity)
            // You can adjust the gain (e.g., multiply by a factor) to control responsiveness
            double rotationSpeed = angle * 60; // Convert angle to angular speed

            return rotationAxis * rotationSpeed;
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
            if(armtype == "bomb")
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





        public static double CalculateAngleBetween(Vector3D vector1, Vector3D vector2)
        {
            // Normalize both vectors to ensure the angle calculation is accurate
            vector1 = Vector3D.Normalize(vector1);
            vector2 = Vector3D.Normalize(vector2);

            // Calculate the dot product of the two vectors
            double dotProduct = Vector3D.Dot(vector1, vector2);

            // Clamp the dot product to avoid numerical issues (e.g., slightly out-of-bounds due to floating-point errors)
            dotProduct = MathHelper.Clamp(dotProduct, -1.0, 1.0);

            // Return the angle in radians
            return Math.Acos(dotProduct);
        }

        void ApplyGyroOverride(Vector3D rotationVector, List<IMyGyro> gyros, MatrixD worldMatrix)
        {
            foreach (var g in gyros)
            {
                var localRotationVec = Vector3D.TransformNormal(rotationVector, Matrix.Transpose(g.WorldMatrix));

                // Negate pitch and yaw adjustments
                float pitchAdjustment = (float)(-localRotationVec.X);
                float yawAdjustment = (float)(-localRotationVec.Y);
                float rollAdjustment = (float)(-localRotationVec.Z);

                g.Enabled = true;
                g.Pitch = pitchAdjustment;
                g.Yaw = yawAdjustment;
                g.Roll = rollAdjustment;
                g.GyroOverride = true;
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

        private int GetBayNumberFromMergeBlock(IMyShipMergeBlock mergeBlock)
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
            public static Vector3D SafeNormalize(Vector3D a)
            {
                if (Vector3D.IsZero(a)) return Vector3D.Zero;
                if (Vector3D.IsUnit(ref a)) return a;
                return Vector3D.Normalize(a);
            }

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

        public void DisplayOnLCD(IMyTextSurface lcd, double distanceToTarget, double startingDistance, double currentVelocity, int _ticks)
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

            //if (timeToTargetSec <= 2.5)
            //{
            //    if (timeToTargetSec > 0.1)
            //    {
            //        int maxExplosionRadius = 20;
            //        double explosionIntensity = (2.5 - (timeToTargetSec - 0.1)) / 2.5;
            //        int currentRadius = (int)(maxExplosionRadius * explosionIntensity);
            //        StringBuilder explosionBuilder = new StringBuilder();
            //        string boomText = "\n                           H A V E   A   N I C E   D A Y!";
            //        explosionBuilder.Append(boomText);

            //        for (int y = -maxExplosionRadius; y <= maxExplosionRadius; y++)
            //        {
            //            for (int x = -maxExplosionRadius; x <= maxExplosionRadius; x++)
            //            {
            //                double distanceFromCenter = Math.Sqrt(x * x + y * y);
            //                if (distanceFromCenter < currentRadius)
            //                {
            //                    if (distanceFromCenter > currentRadius * 0.7 && distanceFromCenter < currentRadius * 0.9)
            //                        explosionBuilder.Append(ColorToChar(255, 69, 0));
            //                    else if (distanceFromCenter > currentRadius * 0.3 && distanceFromCenter < currentRadius * 0.7)
            //                        explosionBuilder.Append(ColorToChar(255, 140, 0));
            //                    else if (distanceFromCenter <= currentRadius * 0.3)
            //                        explosionBuilder.Append(ColorToChar(255, 255, 224));
            //                    else
            //                        explosionBuilder.Append(ColorToChar(255, 255, 0));
            //                }
            //                else
            //                {
            //                    explosionBuilder.Append(' ');
            //                }
            //            }
            //            explosionBuilder.Append('\n');
            //        }
            //        lcd.WriteText(explosionBuilder.ToString(), false);
            //    }
            //    else
            //    {
            //        lcd.WriteText("", false);
            //    }
            //}
            //else
            //{
                // Positioning variables
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
                                        $"  {target}".PadRight(10) + "\n" +
                                        $"  {bottomRightLabel}{startingDistance.ToString("0.0")} m".PadRight(10) + "";
                string progressText = $"  {progressLabel}{distanceBar}".PadRight(25);
                string centerMessage = $"  {scrollingMessage}".PadRight(25);
                string quoteMessage = $"  {scrollingQuote}".PadRight(25);

                // Render the overlapping X's in ASCII

                // ... [Your existing code to create the final output string] ...
                string footerWithLogo = $"+======================================================+";

                // Replace asciiArrow with asciiBoxCrosshair and asciiCircle in the final output
                string output = $"{header}\n{centerMessage}\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n{topSeparator}\n{topLeftText}{topRightText}\n{bottomSeparator}\n{bottomLeftText}\n{progressText}\n{quoteMessage}\n{footerWithLogo}";

                // Display the text on the LCD
                lcd.WriteText(output, false);
            //}
        }

        private static char ColorToChar(int r, int g, int b)
        {
            const double BIT_SPACING = 255.0 / 7.0;
            return (char)(0xe100 + ((int)Math.Round(r / BIT_SPACING) << 6) + ((int)Math.Round(g / BIT_SPACING) << 3) + (int)Math.Round(b / BIT_SPACING));
        }
    }
}
