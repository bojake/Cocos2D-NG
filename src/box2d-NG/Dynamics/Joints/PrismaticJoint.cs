using System;

namespace Box2DNG
{
    public sealed class PrismaticJoint
    {
        public Body BodyA { get; }
        public Body BodyB { get; }
        public Vec2 LocalAnchorA { get; }
        public Vec2 LocalAnchorB { get; }
        public Vec2 LocalAxisA { get; }
        public float ReferenceAngle { get; }
        public bool EnableLimit { get; }
        public float LowerTranslation { get; }
        public float UpperTranslation { get; }
        public bool EnableMotor { get; }
        public float MotorSpeed { get; }
        public float MaxMotorForce { get; }

        private Mat22 _k;
        private Vec2 _perp;
        private Vec2 _axis;
        private float _s1;
        private float _s2;
        private float _a1;
        private float _a2;
        private Vec2 _impulse;
        private float _motorImpulse;
        private float _limitImpulse;
        private float _motorMass;

        public PrismaticJoint(PrismaticJointDef def)
        {
            BodyA = def.BodyA;
            BodyB = def.BodyB;
            LocalAnchorA = def.LocalAnchorA;
            LocalAnchorB = def.LocalAnchorB;
            LocalAxisA = def.LocalAxisA.Normalize();
            ReferenceAngle = def.ReferenceAngle;
            EnableLimit = def.EnableLimit;
            LowerTranslation = def.LowerTranslation;
            UpperTranslation = def.UpperTranslation;
            EnableMotor = def.EnableMotor;
            MotorSpeed = def.MotorSpeed;
            MaxMotorForce = def.MaxMotorForce;
        }

        internal void InitVelocityConstraints(float dt)
        {
            Vec2 rA = Rot.Mul(BodyA.Transform.Q, LocalAnchorA - BodyA.LocalCenter);
            Vec2 rB = Rot.Mul(BodyB.Transform.Q, LocalAnchorB - BodyB.LocalCenter);

            Vec2 d = (BodyB.GetWorldCenter() + rB) - (BodyA.GetWorldCenter() + rA);
            _axis = Rot.Mul(BodyA.Transform.Q, LocalAxisA);
            _perp = new Vec2(-_axis.Y, _axis.X);

            _s1 = Vec2.Cross(d + rA, _perp);
            _s2 = Vec2.Cross(rB, _perp);
            _a1 = Vec2.Cross(d + rA, _axis);
            _a2 = Vec2.Cross(rB, _axis);

            float mA = BodyA.InverseMass;
            float mB = BodyB.InverseMass;
            float iA = BodyA.InverseInertia;
            float iB = BodyB.InverseInertia;

            float k11 = mA + mB + iA * _s1 * _s1 + iB * _s2 * _s2;
            float k12 = iA * _s1 + iB * _s2;
            float k22 = iA + iB;
            if (k22 == 0f)
            {
                // For bodies with fixed rotation, keep the matrix invertible.
                k22 = 1f;
            }
            _k = new Mat22(new Vec2(k11, k12), new Vec2(k12, k22));

            float motorMass = mA + mB + iA * _a1 * _a1 + iB * _a2 * _a2;
            _motorMass = motorMass > 0f ? 1f / motorMass : 0f;

            if (!EnableLimit)
            {
                _limitImpulse = 0f;
            }
            else
            {
                float translation = Vec2.Dot(_axis, d);
                if (translation > LowerTranslation && translation < UpperTranslation)
                {
                    _limitImpulse = 0f;
                }
            }

            if (!EnableMotor)
            {
                _motorImpulse = 0f;
            }

            if (_impulse.X != 0f || _impulse.Y != 0f || _motorImpulse != 0f || _limitImpulse != 0f)
            {
                Vec2 P = _impulse.X * _perp + (_motorImpulse + _limitImpulse) * _axis;
                float LA = _impulse.X * _s1 + _impulse.Y + (_motorImpulse + _limitImpulse) * _a1;
                float LB = _impulse.X * _s2 + _impulse.Y + (_motorImpulse + _limitImpulse) * _a2;

                BodyA.LinearVelocity -= mA * P;
                BodyA.AngularVelocity -= iA * LA;
                BodyB.LinearVelocity += mB * P;
                BodyB.AngularVelocity += iB * LB;
            }
        }

        internal void SolveVelocityConstraints(float dt)
        {
            float mA = BodyA.InverseMass;
            float mB = BodyB.InverseMass;
            float iA = BodyA.InverseInertia;
            float iB = BodyB.InverseInertia;

            Vec2 rA = Rot.Mul(BodyA.Transform.Q, LocalAnchorA - BodyA.LocalCenter);
            Vec2 rB = Rot.Mul(BodyB.Transform.Q, LocalAnchorB - BodyB.LocalCenter);

            Vec2 vA = BodyA.LinearVelocity;
            Vec2 vB = BodyB.LinearVelocity;
            float wA = BodyA.AngularVelocity;
            float wB = BodyB.AngularVelocity;

            Vec2 Cdot1 = new Vec2(
                Vec2.Dot(_perp, vB + Vec2.Cross(wB, rB) - vA - Vec2.Cross(wA, rA)),
                wB - wA);
            Vec2 impulse1 = Solve22(_k, -Cdot1);
            _impulse += impulse1;

            Vec2 P = impulse1.X * _perp;
            float LA = impulse1.X * _s1 + impulse1.Y;
            float LB = impulse1.X * _s2 + impulse1.Y;

            vA -= mA * P;
            wA -= iA * LA;
            vB += mB * P;
            wB += iB * LB;

            if (EnableMotor)
            {
                float Cdot = Vec2.Dot(_axis, vB + Vec2.Cross(wB, rB) - vA - Vec2.Cross(wA, rA));
                float impulse = _motorMass * (MotorSpeed - Cdot);
                float maxImpulse = MaxMotorForce * dt;
                float newImpulse = MathFng.Clamp(_motorImpulse + impulse, -maxImpulse, maxImpulse);
                float applied = newImpulse - _motorImpulse;
                _motorImpulse = newImpulse;

                Vec2 motorP = applied * _axis;
                float motorLA = applied * _a1;
                float motorLB = applied * _a2;

                vA -= mA * motorP;
                wA -= iA * motorLA;
                vB += mB * motorP;
                wB += iB * motorLB;
            }

            if (EnableLimit)
            {
                float translation = Vec2.Dot(_axis, (BodyB.GetWorldCenter() + rB) - (BodyA.GetWorldCenter() + rA));
                float C = 0f;
                if (translation < LowerTranslation)
                {
                    C = translation - LowerTranslation;
                }
                else if (translation > UpperTranslation)
                {
                    C = translation - UpperTranslation;
                }
                else
                {
                    C = 0f;
                }

                if (C != 0f)
                {
                    float Cdot = Vec2.Dot(_axis, vB + Vec2.Cross(wB, rB) - vA - Vec2.Cross(wA, rA));
                    float impulse = -_motorMass * (Cdot + MathF.Min(0f, C) * 0.2f);
                    float prevImpulse = _limitImpulse;
                    if (translation < LowerTranslation)
                    {
                        _limitImpulse = MathF.Min(_limitImpulse + impulse, 0f);
                    }
                    else if (translation > UpperTranslation)
                    {
                        _limitImpulse = MathF.Max(_limitImpulse + impulse, 0f);
                    }
                    float applied = _limitImpulse - prevImpulse;

                    Vec2 limitP = applied * _axis;
                    float limitLA = applied * _a1;
                    float limitLB = applied * _a2;
                    vA -= mA * limitP;
                    wA -= iA * limitLA;
                    vB += mB * limitP;
                    wB += iB * limitLB;
                }
            }

            BodyA.LinearVelocity = vA;
            BodyA.AngularVelocity = wA;
            BodyB.LinearVelocity = vB;
            BodyB.AngularVelocity = wB;
        }

        internal void SolvePositionConstraints()
        {
            float mA = BodyA.InverseMass;
            float mB = BodyB.InverseMass;
            float iA = BodyA.InverseInertia;
            float iB = BodyB.InverseInertia;

            Vec2 rA = Rot.Mul(BodyA.Transform.Q, LocalAnchorA - BodyA.LocalCenter);
            Vec2 rB = Rot.Mul(BodyB.Transform.Q, LocalAnchorB - BodyB.LocalCenter);

            Vec2 d = (BodyB.GetWorldCenter() + rB) - (BodyA.GetWorldCenter() + rA);
            Vec2 axis = Rot.Mul(BodyA.Transform.Q, LocalAxisA);
            Vec2 perp = new Vec2(-axis.Y, axis.X);

            float s1 = Vec2.Cross(d + rA, perp);
            float s2 = Vec2.Cross(rB, perp);
            float a1 = Vec2.Cross(d + rA, axis);
            float a2 = Vec2.Cross(rB, axis);

            float C1 = Vec2.Dot(perp, d);
            float C2 = (BodyB.Transform.Q.Angle - BodyA.Transform.Q.Angle) - ReferenceAngle;

            float k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
            float k12 = iA * s1 + iB * s2;
            float k22 = iA + iB;
            if (k22 == 0f)
            {
                k22 = 1f;
            }
            Mat22 k = new Mat22(new Vec2(k11, k12), new Vec2(k12, k22));
            Vec2 impulse = Solve22(k, new Vec2(-C1, -C2));

            Vec2 P = impulse.X * perp;
            float LA = impulse.X * s1 + impulse.Y;
            float LB = impulse.X * s2 + impulse.Y;

            Vec2 centerA = BodyA.GetWorldCenter() - mA * P;
            Vec2 centerB = BodyB.GetWorldCenter() + mB * P;
            float angleA = BodyA.Transform.Q.Angle - iA * LA;
            float angleB = BodyB.Transform.Q.Angle + iB * LB;
            BodyA.SetTransformFromCenter(centerA, angleA);
            BodyB.SetTransformFromCenter(centerB, angleB);

            if (EnableLimit)
            {
                float translation = Vec2.Dot(axis, d);
                float C = 0f;
                if (translation < LowerTranslation)
                {
                    C = translation - LowerTranslation;
                }
                else if (translation > UpperTranslation)
                {
                    C = translation - UpperTranslation;
                }
                else
                {
                    return;
                }

                float kAxis = mA + mB + iA * a1 * a1 + iB * a2 * a2;
                float impulseAxis = kAxis > 0f ? -C / kAxis : 0f;
                Vec2 PA = impulseAxis * axis;
                float LA2 = impulseAxis * a1;
                float LB2 = impulseAxis * a2;

                Vec2 centerA2 = BodyA.GetWorldCenter() - mA * PA;
                Vec2 centerB2 = BodyB.GetWorldCenter() + mB * PA;
                float angleA2 = BodyA.Transform.Q.Angle - iA * LA2;
                float angleB2 = BodyB.Transform.Q.Angle + iB * LB2;
                BodyA.SetTransformFromCenter(centerA2, angleA2);
                BodyB.SetTransformFromCenter(centerB2, angleB2);
            }
        }

        private static Vec2 Solve22(Mat22 m, Vec2 b)
        {
            float a11 = m.Ex.X;
            float a12 = m.Ey.X;
            float a21 = m.Ex.Y;
            float a22 = m.Ey.Y;
            float det = a11 * a22 - a12 * a21;
            if (det != 0f)
            {
                det = 1f / det;
            }
            return new Vec2(det * (a22 * b.X - a12 * b.Y), det * (a11 * b.Y - a21 * b.X));
        }
    }
}
