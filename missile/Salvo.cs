using System;
using VRageMath;

namespace IngameScript
{
    partial class Program
    {
        static void BuildPerpBasis(Vector3D forward, out Vector3D u, out Vector3D v)
        {
            Vector3D candidate = Math.Abs(forward.Y) < 0.9 ? Vector3D.Up : Vector3D.Right;
            u = VectorMath.SafeNormalize(Vector3D.Cross(forward, candidate));
            v = Vector3D.Cross(forward, u);
        }

        Vector3D ComputeConeApproachOffset(Vector3D nominalApproachFrom, int salvoIndex, int salvoSize, double coneDegrees, double approachRadius)
        {
            if (salvoSize <= 0 || approachRadius <= 0.0)
            {
                return Vector3D.Zero;
            }

            Vector3D baseDir = VectorMath.SafeNormalize(nominalApproachFrom);
            if (Vector3D.IsZero(baseDir))
            {
                return Vector3D.Zero;
            }

            double alpha = MathHelper.ToRadians(coneDegrees);
            double theta = 2.0 * Math.PI * salvoIndex / Math.Max(salvoSize, 1);

            Vector3D u, v;
            BuildPerpBasis(baseDir, out u, out v);

            Vector3D approachDir = Math.Cos(alpha) * baseDir
                                 + Math.Sin(alpha) * (Math.Cos(theta) * u + Math.Sin(theta) * v);

            return VectorMath.SafeNormalize(approachDir) * approachRadius;
        }

        Vector3D ComputeInterceptPoint(Vector3D missilePos, Vector3D missileVel, Vector3D targetPos, Vector3D targetVel, double missileAccel)
        {
            Vector3D r = targetPos - missilePos;
            double distance = r.Length();
            if (distance < 1.0)
            {
                return targetPos;
            }

            double missileSpeed = missileVel.Length();
            double t = distance / Math.Max(missileSpeed + 50.0, 100.0);

            for (int i = 0; i < 3; i++)
            {
                Vector3D predicted = targetPos + targetVel * t;
                double d = (predicted - missilePos).Length();
                double avgSpeed = missileSpeed + 0.5 * missileAccel * t;
                t = d / Math.Max(avgSpeed, 100.0);
            }

            return targetPos + targetVel * t;
        }
    }
}
