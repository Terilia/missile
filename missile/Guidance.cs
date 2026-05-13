using Sandbox.ModAPI.Ingame;
using System;
using System.Collections.Generic;
using VRageMath;

namespace IngameScript
{
    partial class Program
    {
        public static Vector3D GravityCompensation(double missileAcceleration, Vector3D desiredDirection, Vector3D gravity)
        {
            Vector3D directionNorm = VectorMath.SafeNormalize(desiredDirection);
            Vector3D gravityComp = -VectorMath.Rejection(gravity, desiredDirection);

            double diffSq = missileAcceleration * missileAcceleration - gravityComp.LengthSquared();
            if (diffSq < 0)
            {
                return desiredDirection - gravity;
            }

            return directionNorm * Math.Sqrt(diffSq) + gravityComp;
        }

        public static Vector3D GetRotationVector(Vector3D desiredForwardVector, Vector3D? desiredUpVector, MatrixD worldMatrix)
        {
            var transposedWm = MatrixD.Transpose(worldMatrix);
            var forwardVector = Vector3D.Rotate(VectorMath.SafeNormalize(desiredForwardVector), transposedWm);

            Vector3D leftVector = Vector3D.Zero;
            if (desiredUpVector.HasValue)
            {
                desiredUpVector = Vector3D.Rotate(desiredUpVector.Value, transposedWm);
                leftVector = Vector3D.Cross(desiredUpVector.Value, forwardVector);
            }

            Vector3D axis;
            double angle;
            if (!desiredUpVector.HasValue || Vector3D.IsZero(leftVector))
            {
                axis = new Vector3D(-forwardVector.Y, forwardVector.X, 0);
                angle = Math.Acos(MathHelper.Clamp(-forwardVector.Z, -1.0, 1.0));
            }
            else
            {
                leftVector = VectorMath.SafeNormalize(leftVector);
                var upVector = Vector3D.Cross(forwardVector, leftVector);
                var targetOrientation = new MatrixD()
                {
                    Forward = forwardVector,
                    Left = leftVector,
                    Up = upVector,
                };

                axis = new Vector3D(targetOrientation.M32 - targetOrientation.M23,
                                    targetOrientation.M13 - targetOrientation.M31,
                                    targetOrientation.M21 - targetOrientation.M12);

                double trace = targetOrientation.M11 + targetOrientation.M22 + targetOrientation.M33;
                angle = Math.Acos(MathHelper.Clamp((trace - 1) * 0.5, -1.0, 1.0));
            }

            Vector3D rotationVectorPYR;
            if (Vector3D.IsZero(axis))
            {
                angle = forwardVector.Z < 0 ? 0 : Math.PI;
                rotationVectorPYR = new Vector3D(0, angle, 0);
            }
            else
            {
                rotationVectorPYR = VectorMath.SafeNormalize(axis) * angle;
            }

            return rotationVectorPYR;
        }

        public static void ApplyGyroOverride(double pitchSpeed, double yawSpeed, double rollSpeed, List<IMyGyro> gyroList, MatrixD worldMatrix)
        {
            var rotationVec = new Vector3D(pitchSpeed, yawSpeed, rollSpeed);
            var relativeRotationVec = Vector3D.TransformNormal(rotationVec, worldMatrix);

            foreach (var thisGyro in gyroList)
            {
                var transformedRotationVec = Vector3D.TransformNormal(relativeRotationVec, Matrix.Transpose(thisGyro.WorldMatrix));

                thisGyro.Pitch = (float)transformedRotationVec.X;
                thisGyro.Yaw = (float)transformedRotationVec.Y;
                thisGyro.Roll = (float)transformedRotationVec.Z;
                thisGyro.GyroOverride = true;
            }
        }

        void GyroTurn6(Vector3D TARGETVECTOR, double GAIN, double DAMPINGGAIN, IMyTerminalBlock REF, IMyGyro GYRO, double YawPrev, double PitchPrev, out double NewPitch, out double NewYaw, double distanceToTarget)
        {
            NewYaw = 0;
            NewPitch = 0;

            Vector3D ShipUp = REF.WorldMatrix.Up;
            Vector3D ShipForward = REF.WorldMatrix.Backward;

            Quaternion Quat_Two = Quaternion.CreateFromForwardUp(ShipForward, ShipUp);
            var InvQuat = Quaternion.Inverse(Quat_Two);

            Vector3D DirectionVector = TARGETVECTOR;
            Vector3D RCReferenceFrameVector = Vector3D.Transform(DirectionVector, InvQuat);

            double ShipForwardAzimuth = 0;
            double ShipForwardElevation = 0;
            Vector3D.GetAzimuthAndElevation(RCReferenceFrameVector, out ShipForwardAzimuth, out ShipForwardElevation);

            NewYaw = ShipForwardAzimuth;
            NewPitch = ShipForwardElevation;

            ShipForwardAzimuth = ShipForwardAzimuth + DAMPINGGAIN * ((ShipForwardAzimuth - YawPrev) / tickTime);
            ShipForwardElevation = ShipForwardElevation + DAMPINGGAIN * ((ShipForwardElevation - PitchPrev) / tickTime);

            var REF_Matrix = MatrixD.CreateWorld(REF.GetPosition(), (Vector3)ShipForward, (Vector3)ShipUp).GetOrientation();
            var Vector = Vector3.Transform(new Vector3D(ShipForwardElevation, ShipForwardAzimuth, 0), REF_Matrix);
            var TRANS_VECT = Vector3.Transform(Vector, Matrix.Transpose(GYRO.WorldMatrix.GetOrientation()));

            if (double.IsNaN(TRANS_VECT.X) || double.IsNaN(TRANS_VECT.Y) || double.IsNaN(TRANS_VECT.Z))
                return;

            GYRO.Pitch = (float)MathHelper.Clamp((-TRANS_VECT.X) * GAIN, -500, 500);
            GYRO.Yaw = (float)MathHelper.Clamp((-TRANS_VECT.Y) * GAIN, -500, 500);
            GYRO.Roll = (float)MathHelper.Clamp((-TRANS_VECT.Z) * GAIN, -500, 500);
            GYRO.GyroOverride = true;
        }
    }
}
