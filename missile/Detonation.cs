using Sandbox.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame;
using VRageMath;

namespace IngameScript
{
    partial class Program
    {
        bool PerformRaycastCheck()
        {
            if (_proxCam == null)
            {
                _proxCam = GridTerminalSystem.GetBlockWithName("ProxCam") as IMyCameraBlock;
                if (_proxCam == null) return false;
                _proxCam.EnableRaycast = true;
            }
            if (_proxCam.CanScan(3.0))
            {
                MyDetectedEntityInfo hitInfo = _proxCam.Raycast(3.0);
                if (hitInfo.HitPosition.HasValue)
                {
                    return true;
                }
            }
            return false;
        }

        bool SegmentIntersectsSphere(Vector3D segStart, Vector3D segEnd, Vector3D center, double radius)
        {
            Vector3D seg = segEnd - segStart;
            double segLenSq = seg.LengthSquared();
            if (segLenSq < 1e-6)
            {
                return Vector3D.DistanceSquared(segStart, center) <= radius * radius;
            }
            double t = Vector3D.Dot(center - segStart, seg) / segLenSq;
            t = MathHelper.Clamp(t, 0.0, 1.0);
            Vector3D closest = segStart + t * seg;
            return Vector3D.DistanceSquared(closest, center) <= radius * radius;
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
    }
}
