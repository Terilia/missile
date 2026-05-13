using VRageMath;

namespace IngameScript
{
    partial class Program
    {
        bool TryParseGPS(string gps, out Vector3D position)
        {
            position = Vector3D.Zero;
            if (string.IsNullOrWhiteSpace(gps)) return false;

            string[] parts = gps.Split(':');
            if (parts.Length < 6) return false;

            double x, y, z;
            if (double.TryParse(parts[2], out x) &&
                double.TryParse(parts[3], out y) &&
                double.TryParse(parts[4], out z))
            {
                position = new Vector3D(x, y, z);
                return true;
            }
            return false;
        }

        void AddLoftedTrajectoryWaypoints(double fraction = 0.5, double loftHeight = 9000)
        {
            if (_waypoints.Count >= 1 && _remoteControl != null)
            {
                Vector3D currentPosition = _remoteControl.GetPosition();
                Vector3D finalPoint = _waypoints[0];

                Vector3D gravityVector = _remoteControl.GetNaturalGravity();

                if (gravityVector.LengthSquared() == 0)
                {
                    Echo("No gravity detected. Cannot calculate lofted trajectory.");
                    return;
                }

                Vector3D upDirection = -VectorMath.SafeNormalize(gravityVector);

                Vector3D directionToTarget = finalPoint - currentPosition;

                Vector3D horizontalDirection = VectorMath.Rejection(directionToTarget, upDirection);

                double horizontalDistance = horizontalDirection.Length();

                if (horizontalDistance > 0)
                {
                    Vector3D horizontalDirectionNormalized = horizontalDirection / horizontalDistance;

                    Vector3D waypoint = currentPosition + horizontalDirectionNormalized * (horizontalDistance * fraction);

                    waypoint += upDirection * loftHeight;

                    _waypoints.Insert(0, waypoint);
                }
                else
                {
                    Vector3D waypoint = currentPosition + upDirection * loftHeight;
                    _waypoints.Insert(0, waypoint);
                }
            }
        }
    }
}
