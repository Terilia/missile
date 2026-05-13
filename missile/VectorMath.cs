using System;
using VRageMath;

namespace IngameScript
{
    partial class Program
    {
        public static double Vector_Projection_Scalar(Vector3D IN, Vector3D Axis_norm)
        {
            double OUT = Vector3D.Dot(IN, Axis_norm);
            if (double.IsNaN(OUT))
            {
                OUT = 0;
            }
            return OUT;
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

            public static Vector3D Rejection(Vector3D a, Vector3D b)
            {
                if (Vector3D.IsZero(a) || Vector3D.IsZero(b)) return Vector3D.Zero;
                return a - Projection(a, b);
            }

            public static Vector3D Reject(Vector3D a, Vector3D b)
            {
                return Rejection(a, b);
            }

            public static double AngleBetween(Vector3D a, Vector3D b)
            {
                if (Vector3D.IsZero(a) || Vector3D.IsZero(b)) return 0;
                double cos = a.Dot(b) / Math.Sqrt(a.LengthSquared() * b.LengthSquared());
                return Math.Acos(MathHelper.Clamp(cos, -1.0, 1.0));
            }
        }
    }
}
