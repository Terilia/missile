using Sandbox.ModAPI.Ingame;
using SpaceEngineers.Game.ModAPI.Ingame;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using VRageMath;

namespace IngameScript
{
    partial class Program
    {
        void Initialize()
        {
            _salvoIndex = Math.Max(0, _bayNumber - 1);
            string channel = $"JETOS_MSL_{_bayNumber}";
            _targetListener = IGC.RegisterBroadcastListener(channel);
            _targetListener.SetMessageCallback(channel);
        }

        void CheckForGPSAndStart()
        {
            programmableBlock = FindLauncherProgrammableBlock();
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
                if (line.StartsWith("Cone:"))
                {
                    double v;
                    if (double.TryParse(line.Substring(line.IndexOf(':') + 1).Trim(), out v))
                        _coneDegrees = v;
                }
                if (line.StartsWith("SalvoSize:"))
                {
                    int v;
                    if (int.TryParse(line.Substring(line.IndexOf(':') + 1).Trim(), out v))
                        _salvoSize = Math.Max(1, v);
                }
                if (line.StartsWith("SalvoIndex:"))
                {
                    int v;
                    if (int.TryParse(line.Substring(line.IndexOf(':') + 1).Trim(), out v))
                        _salvoIndex = Math.Max(0, v);
                }
                if (line.StartsWith("ApproachRadius:"))
                {
                    double v;
                    if (double.TryParse(line.Substring(line.IndexOf(':') + 1).Trim(), out v))
                        _approachRadius = Math.Max(0, v);
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

                            if (_coneDegrees > 0.0 && _salvoSize > 1 && _approachRadius > 0.0)
                            {
                                Vector3D nominal = Me.GetPosition() - targetPosition;
                                _approachOffset = ComputeConeApproachOffset(nominal, _salvoIndex, _salvoSize, _coneDegrees, _approachRadius);
                            }

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

        IMyProgrammableBlock FindLauncherProgrammableBlock()
        {
            var exact = GridTerminalSystem.GetBlockWithName("JETOS Programmable Block") as IMyProgrammableBlock;
            if (exact != null && exact != Me) return exact;

            var programmableBlocks = new List<IMyProgrammableBlock>();
            GridTerminalSystem.GetBlocksOfType(programmableBlocks, block => block != Me);

            string bayPrefix = _bayNumber.ToString() + ":";
            for (int i = 0; i < programmableBlocks.Count; i++)
            {
                string data = programmableBlocks[i].CustomData;
                if (data.IndexOf(bayPrefix, StringComparison.Ordinal) >= 0
                    && (data.IndexOf("Topdown:", StringComparison.Ordinal) >= 0
                        || data.IndexOf("AntiAir:", StringComparison.Ordinal) >= 0
                        || data.IndexOf("Cached:", StringComparison.Ordinal) >= 0))
                {
                    return programmableBlocks[i];
                }
            }

            for (int i = 0; i < programmableBlocks.Count; i++)
            {
                if (programmableBlocks[i].CustomData.IndexOf(bayPrefix, StringComparison.Ordinal) >= 0)
                    return programmableBlocks[i];
            }

            return null;
        }

        void InitializeThrusters()
        {
            foreach (var thruster in _thrusters)
            {
                thruster.ThrustOverridePercentage = 1f;
                thruster.Enabled = true;
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
    }
}
