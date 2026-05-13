using Sandbox.ModAPI.Ingame;
using SpaceEngineers.Game.ModAPI.Ingame;
using System.Collections.Generic;
using VRage.Game.ModAPI.Ingame;
using VRageMath;

namespace IngameScript
{
    partial class Program
    {
        const double WAYPOINT_THRESHOLD = 550.0;
        const double DETONATION_DISTANCE = 8.0;
        const float tickTime = 1f / 60f;
        const double UpdatesPerSecond = 60.0;

        double _navConstant = 4.0;
        double _navAccelConstant = 1.5;

        bool isTopdown = false;
        bool _antiairmode = false;
        bool _isStarted = false;
        bool _isInitialized = false;
        bool firstrun = true;

        int _ticks = 0;
        int _currentWaypointIndex = 0;
        int _bayNumber;
        int _cooldown = 0;
        bool _hasRadarLock = false;

        List<Vector3D> _waypoints = new List<Vector3D>();
        List<IMyGyro> _gyros = new List<IMyGyro>();
        List<IMyThrust> _thrusters = new List<IMyThrust>();
        List<IMyWarhead> _warheads = new List<IMyWarhead>();
        List<IMyTerminalBlock> blocks = new List<IMyTerminalBlock>();

        IMyRemoteControl _remoteControl;
        IMyTextPanel lcdMain;
        IMyShipMergeBlock _mergeBlock;
        IMySensorBlock _sensor;
        IMyProgrammableBlock programmableBlock;
        IMyLargeGatlingTurret radar;
        IMyCameraBlock _proxCam;

        Vector3D targetPosition;
        Vector3D targetvelocity = Vector3D.Zero;
        Vector3D? _lastTargetVelocity = null;
        MyDetectedEntityInfo detectedEntity;
        double startingDistance = 0;

        double MissileThrust = 0;
        double MissileAccel = 0;
        double MissileMass = 0;

        double PREV_Yaw = 0;
        double PREV_Pitch = 0;

        double _prevDistance = double.MaxValue;
        int _lastCustomDataHash = 0;

        Vector3D _approachOffset = Vector3D.Zero;
        int _salvoIndex = 0;
        int _salvoSize = 1;
        double _coneDegrees = 0.0;
        double _approachRadius = 500.0;
        const double TERMINAL_SWITCH_DISTANCE = 200.0;
        const double MAX_SPEED = 420.0;
        const double SPEED_CAP_BAND = 20.0;

        IMyBroadcastListener _targetListener;
        bool _igcActive = false;
        const string STATUS_CHANNEL = "JETOS_MSL_STAT";
        const int TELEMETRY_TICKS = 5;

        readonly Queue<string> _igcLog = new Queue<string>();
        int _lastLogTick = -9999;
        const int LOG_MAX_LINES = 3;
        const int LOG_THROTTLE_TICKS = 30;
    }
}
