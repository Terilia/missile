using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using System;
using System.Collections.Generic;
using VRage;
using VRage.Game.ModAPI.Ingame;
using VRageMath;

namespace IngameScript
{
    partial class Program : MyGridProgram
    {
        public Program()
        {
            Runtime.UpdateFrequency = UpdateFrequency.Update1;
        }

        public void Save() { }

        public void Main(string argument, UpdateType updateSource)
        {
            Echo(_ticks.ToString());

            if (argument == "detonate")
            {
                DetonateWarheads();
                return;
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
                return;
            }

            _ticks++;

            if (_ticks < 100)
            {
                RunStagedInit();
                return;
            }

            RunGuidanceLoop();
        }

        void RunStagedInit()
        {
            if (_ticks < 10 && _ticks > 4)
            {
                _remoteControl = GridTerminalSystem.GetBlockWithName("Remote Control Missile") as IMyRemoteControl;
                if (_remoteControl == null)
                {
                    Echo("[ERROR] Remote Control not found!");
                    throw new Exception("Remote Control not found!");
                }

                lcdMain = GridTerminalSystem.GetBlockWithName("Holo LCD") as IMyTextPanel;
                radar = GridTerminalSystem.GetBlockWithName("Radar") as IMyLargeGatlingTurret;
                _proxCam = GridTerminalSystem.GetBlockWithName("ProxCam") as IMyCameraBlock;
                if (_proxCam != null) _proxCam.EnableRaycast = true;

                _sensor = GridTerminalSystem.GetBlockWithName("Sensor") as IMySensorBlock;
                if (_sensor != null) _sensor.Enabled = true;

                GridTerminalSystem.GetBlocksOfType(_warheads);
                GridTerminalSystem.GetBlocksOfType(_gyros);

                GridTerminalSystem.GetBlocksOfType(_thrusters);
                {
                    List<IMyThrust> filteredThrusters = new List<IMyThrust>();
                    foreach (var t in _thrusters)
                    {
                        if (t.CustomName.Contains("Sci-Fi"))
                        {
                            filteredThrusters.Add(t);
                        }
                    }
                    if (filteredThrusters.Count == 0)
                    {
                        Echo("No Sci-Fi thrusters found!");
                        return;
                    }
                    _thrusters = filteredThrusters;
                    InitializeThrusters();
                }

                startingDistance = Vector3D.Distance(_remoteControl.GetPosition(), _waypoints[_waypoints.Count - 1]);

                if (firstrun && isTopdown && !_antiairmode)
                {
                    AddLoftedTrajectoryWaypoints();
                    firstrun = false;
                    Echo("[WAYPOINTS] Lofted trajectory waypoints added.");
                }

                if (radar != null)
                {
                    radar.Enabled = true;
                    radar.TargetEnemies = true;
                    radar.TargetStations = true;
                    radar.EnableIdleRotation = false;
                }

                if (_remoteControl != null)
                {
                    _remoteControl.IsMainCockpit = true;
                    _remoteControl.DampenersOverride = false;
                }
            }

            if (_ticks == 16)
            {
                GridTerminalSystem.GetBlocks(blocks);
                foreach (var block in blocks)
                {
                    var functional = block as IMyFunctionalBlock;
                    if (functional != null) functional.Enabled = true;
                }
            }

            if (_ticks > 15 && _ticks < 30 && _remoteControl != null && _thrusters.Count > 0)
            {
                Vector3D toTarget = targetPosition - _remoteControl.GetPosition();
                Vector3D desiredForward = VectorMath.SafeNormalize(toTarget);
                Vector3D gravity = _remoteControl.GetNaturalGravity();
                if (gravity.LengthSquared() > 1e-3)
                {
                    desiredForward = VectorMath.SafeNormalize(desiredForward + VectorMath.SafeNormalize(-gravity) * 0.6);
                }
                Vector3D? desiredUp = gravity.LengthSquared() > 1e-3 ? (Vector3D?)(-gravity) : null;
                MatrixD startupMatrix = BuildMissileMatrix();
                Vector3D rotPYR = GetRotationVector(desiredForward, desiredUp, startupMatrix);
                ApplyGyroOverride(rotPYR.X * 3.0, rotPYR.Y * 3.0, rotPYR.Z * 1.5, _gyros, startupMatrix);
            }

            if (_remoteControl != null)
            {
                Vector3D p = _remoteControl.GetPosition();
                SendStatus(p, _remoteControl.GetShipVelocities().LinearVelocity, Vector3D.Distance(p, targetPosition), false);
            }
        }

        void LogIgc(string body)
        {
            if (_ticks - _lastLogTick < LOG_THROTTLE_TICKS) return;
            _lastLogTick = _ticks;
            double tSec = _ticks / UpdatesPerSecond;
            _igcLog.Enqueue($"{tSec,6:F1}s {body}");
            while (_igcLog.Count > LOG_MAX_LINES) _igcLog.Dequeue();
        }

        void SendStatus(Vector3D pos, Vector3D vel, double distance, bool acquired)
        {
            if (_bayNumber < 0 || _ticks % TELEMETRY_TICKS != 0) return;
            double speed = vel.Length();
            double eta = speed > 1.0 ? distance / speed : -1.0;
            IGC.SendBroadcastMessage(STATUS_CHANNEL, MyTuple.Create(_bayNumber, pos, vel, eta, acquired));
        }

        MatrixD BuildMissileMatrix()
        {
            IMyThrust refThruster = _thrusters[0];
            Vector3D fwd = VectorMath.SafeNormalize(refThruster.WorldMatrix.Backward);
            Vector3D up = refThruster.WorldMatrix.Up;
            return MatrixD.CreateWorld(refThruster.GetPosition(), fwd, up);
        }

        void RunGuidanceLoop()
        {
            Vector3D currentPos = _remoteControl.GetPosition();
            Vector3D currentVelocity = _remoteControl.GetShipVelocities().LinearVelocity;
            MatrixD missileMatrix = BuildMissileMatrix();
            Vector3D gravity = _remoteControl.GetNaturalGravity();

            MissileMass = _remoteControl.CalculateShipMass().PhysicalMass;
            MissileThrust = 0;
            foreach (var t in _thrusters)
            {
                if (t.IsFunctional && !t.Closed)
                {
                    MissileThrust += t.MaxEffectiveThrust;
                }
            }
            MissileAccel = MissileThrust / Math.Max(MissileMass, 1.0);

            UpdateTargetState(currentPos);

            double distanceToWaypoint = Vector3D.Distance(currentPos, _waypoints[_currentWaypointIndex]);
            if (_currentWaypointIndex < _waypoints.Count - 1 && distanceToWaypoint <= WAYPOINT_THRESHOLD)
            {
                _currentWaypointIndex++;
            }

            bool onFinalLeg = _currentWaypointIndex >= _waypoints.Count - 1;
            Vector3D effectiveTargetPos = _waypoints[Math.Min(_currentWaypointIndex, _waypoints.Count - 1)];
            Vector3D effectiveTargetVel = onFinalLeg ? targetvelocity : Vector3D.Zero;
            double distanceToTarget = Vector3D.Distance(currentPos, effectiveTargetPos);
            SendStatus(currentPos, currentVelocity, distanceToTarget, _hasRadarLock || _igcActive);

            if (onFinalLeg && distanceToTarget < 500 && PerformRaycastCheck())
            {
                DetonateWarheads();
                return;
            }

            if (onFinalLeg)
            {
                if (distanceToTarget <= DETONATION_DISTANCE)
                {
                    DetonateWarheads();
                    return;
                }
                Vector3D prevPos = currentPos - currentVelocity * tickTime;
                if (SegmentIntersectsSphere(prevPos, currentPos, effectiveTargetPos, DETONATION_DISTANCE))
                {
                    DetonateWarheads();
                    return;
                }
                if (_prevDistance < 50 && distanceToTarget > _prevDistance && _prevDistance < double.MaxValue)
                {
                    DetonateWarheads();
                    return;
                }
            }
            _prevDistance = distanceToTarget;

            Vector3D aimPoint = onFinalLeg
                ? ComputeInterceptPoint(currentPos, currentVelocity, effectiveTargetPos, effectiveTargetVel, MissileAccel)
                : effectiveTargetPos;

            if (onFinalLeg && distanceToTarget > TERMINAL_SWITCH_DISTANCE && _approachOffset.LengthSquared() > 1.0)
            {
                double fade = MathHelper.Clamp((distanceToTarget - TERMINAL_SWITCH_DISTANCE) / _approachRadius, 0.0, 1.0);
                aimPoint = aimPoint + _approachOffset * fade;
            }

            Vector3D desiredDirection = ComputeGuidance(
                currentPos, currentVelocity, aimPoint, Vector3D.Zero, gravity);

            ApplyControl(desiredDirection, distanceToTarget);

            ApplyThrust(desiredDirection, missileMatrix, currentVelocity);

            if (lcdMain != null && _ticks % 3 == 0)
            {
                DisplayOnLCD(lcdMain, distanceToTarget, startingDistance, currentVelocity.Length(), _ticks, currentVelocity);
            }
        }

        void UpdateTargetState(Vector3D currentPos)
        {
            _igcActive = false;
            _hasRadarLock = false;
            if (_targetListener != null)
            {
                bool gotLatestTarget = false;
                Vector3D latestPos = Vector3D.Zero;
                Vector3D latestVel = Vector3D.Zero;
                Vector3D latestApproach = Vector3D.Zero;
                while (_targetListener.HasPendingMessage)
                {
                    MyIGCMessage msg = _targetListener.AcceptMessage();
                    if (msg.Data is MyTuple<Vector3D, Vector3D, Vector3D>)
                    {
                        var t = (MyTuple<Vector3D, Vector3D, Vector3D>)msg.Data;
                        latestPos = t.Item1;
                        latestVel = t.Item2;
                        latestApproach = t.Item3;
                        gotLatestTarget = true;
                    }
                }
                if (gotLatestTarget)
                {
                    targetPosition = latestPos;
                    targetvelocity = latestVel;
                    _approachOffset = latestApproach;
                    _igcActive = true;
                    LogIgc($"TGT {Vector3D.Distance(currentPos, targetPosition),5:F0}m v{targetvelocity.Length(),3:F0}");
                }
            }

            bool gotRadarLock = false;
            if (radar != null)
            {
                detectedEntity = radar.GetTargetedEntity();
                gotRadarLock = !detectedEntity.IsEmpty();
                _hasRadarLock = gotRadarLock;

                if (gotRadarLock)
                {
                    targetPosition = detectedEntity.Position;
                    targetvelocity = detectedEntity.Velocity;
                    _cooldown = 0;
                }
                else
                {
                    double distToLast = Vector3D.Distance(currentPos, targetPosition);
                    if (_cooldown <= 0 && distToLast < 6000)
                    {
                        radar.Enabled = true;
                        radar.ShootOnce();
                        _cooldown = 150;
                    }
                    else
                    {
                        _cooldown--;
                        if (_cooldown < 0) _cooldown = 0;
                    }
                    if (!_igcActive) targetvelocity = Vector3D.Zero;
                }
            }

            if (!gotRadarLock && !_igcActive && _antiairmode && programmableBlock != null)
            {
                string customData = programmableBlock.CustomData;
                int hash = customData.GetHashCode();
                if (hash != _lastCustomDataHash)
                {
                    _lastCustomDataHash = hash;
                    string[] lines = customData.Split(new[] { '\n', '\r' }, StringSplitOptions.RemoveEmptyEntries);
                    foreach (string line in lines)
                    {
                        if (line.StartsWith("Cached:"))
                        {
                            string gpsData = line.Substring(line.IndexOf(':') + 1).Trim();
                            if (!string.IsNullOrEmpty(gpsData))
                            {
                                Vector3D parsed;
                                if (TryParseGPS(gpsData, out parsed))
                                {
                                    targetPosition = parsed;
                                }
                            }
                            break;
                        }
                    }
                }
            }

            if (_antiairmode && _waypoints.Count > 0)
            {
                _waypoints[_waypoints.Count - 1] = targetPosition;
            }
        }

        Vector3D ComputeGuidance(
            Vector3D missilePos,
            Vector3D missileVel,
            Vector3D targetPos,
            Vector3D targetVel,
            Vector3D gravity)
        {
            Vector3D missileToTarget = targetPos - missilePos;
            Vector3D missileToTargetNorm = VectorMath.SafeNormalize(missileToTarget);
            Vector3D relativeVelocity = targetVel - missileVel;

            Vector3D targetAcceleration = Vector3D.Zero;
            if (_lastTargetVelocity.HasValue)
            {
                targetAcceleration = (targetVel - _lastTargetVelocity.Value) * UpdatesPerSecond;
            }
            _lastTargetVelocity = targetVel;

            Vector3D lateralTargetAccel = targetAcceleration
                - Vector3D.Dot(targetAcceleration, missileToTargetNorm) * missileToTargetNorm;

            Vector3D omega = Vector3D.Cross(missileToTarget, relativeVelocity)
                / Math.Max(missileToTarget.LengthSquared(), 1);

            Vector3D lateralAccel = _navConstant * relativeVelocity.Length() * Vector3D.Cross(omega, missileToTargetNorm)
                + _navAccelConstant * lateralTargetAccel;

            double missileAccelSq = MissileAccel * MissileAccel;
            double diff = missileAccelSq - Math.Min(missileAccelSq, lateralAccel.LengthSquared());
            Vector3D pointingVector = lateralAccel + Math.Sqrt(diff) * missileToTargetNorm;

            if (gravity.LengthSquared() > 1e-3)
            {
                pointingVector = GravityCompensation(MissileAccel, pointingVector, gravity);
            }

            return VectorMath.SafeNormalize(pointingVector);
        }

        void ApplyControl(Vector3D desiredDirection, double distanceToTarget)
        {
            double gainMultiplier = Math.Min(1.0, distanceToTarget / 500.0);
            double adaptiveGain = 18.0 * Math.Max(0.6, gainMultiplier);
            double adaptiveDamping = 0.3 * (1.0 + (500.0 / Math.Max(distanceToTarget, 50.0)));

            double Yaw = 0;
            double Pitch = 0;
            foreach (var gyro in _gyros)
            {
                GyroTurn6(desiredDirection, adaptiveGain, adaptiveDamping, _thrusters[0], gyro, PREV_Yaw, PREV_Pitch, out Pitch, out Yaw, distanceToTarget);
            }
            PREV_Yaw = Yaw;
            PREV_Pitch = Pitch;
        }

        void ApplyThrust(Vector3D desiredDirection, MatrixD missileMatrix, Vector3D currentVelocity)
        {
            if (_thrusters.Count == 0)
            {
                return;
            }

            double headingDeviation = Vector3D.Dot(missileMatrix.Forward, desiredDirection);
            double thrustPower = MathHelper.Clamp(headingDeviation, 0.5, 1.0);

            double forwardSpeed = Vector3D.Dot(currentVelocity, missileMatrix.Forward);
            double speedFactor = MathHelper.Clamp((MAX_SPEED - forwardSpeed) / SPEED_CAP_BAND, 0.0, 1.0);
            thrustPower *= speedFactor;

            foreach (var thruster in _thrusters)
            {
                float targetOverride = (float)(thruster.MaxEffectiveThrust * thrustPower);
                if (Math.Abs(thruster.ThrustOverride - targetOverride) > 0.01f)
                {
                    thruster.ThrustOverride = targetOverride;
                }
            }
        }
    }
}
