namespace Box2DNG
{
    public sealed partial class World
    {
        private float GetGearCoordinateA(ref GearJointData joint)
        {
            if (joint.TypeA == GearJointKind.Revolute)
            {
                int jIndex = _revoluteJointIndexById[joint.JointAId];
                ref RevoluteJointData rev = ref _revoluteJointsData[jIndex];
                return (_bodyRotations[joint.BodyB].Angle - _bodyRotations[joint.BodyA].Angle) - rev.ReferenceAngle;
            }

            int pIndex = _prismaticJointIndexById[joint.JointAId];
            ref PrismaticJointData pri = ref _prismaticJointsData[pIndex];
            Vec2 axis = Rot.Mul(_bodyRotations[joint.BodyA], pri.LocalAxisA);
            Vec2 rA = Rot.Mul(_bodyRotations[joint.BodyA], pri.LocalAnchorA - _bodyLocalCenters[joint.BodyA]);
            Vec2 rB = Rot.Mul(_bodyRotations[joint.BodyB], pri.LocalAnchorB - _bodyLocalCenters[joint.BodyB]);
            Vec2 d = (_bodyPositions[joint.BodyB] + rB) - (_bodyPositions[joint.BodyA] + rA);
            return Vec2.Dot(axis, d);
        }

        private float GetGearCoordinateB(ref GearJointData joint)
        {
            if (joint.TypeB == GearJointKind.Revolute)
            {
                int jIndex = _revoluteJointIndexById[joint.JointBId];
                ref RevoluteJointData rev = ref _revoluteJointsData[jIndex];
                return (_bodyRotations[joint.BodyD].Angle - _bodyRotations[joint.BodyC].Angle) - rev.ReferenceAngle;
            }

            int pIndex = _prismaticJointIndexById[joint.JointBId];
            ref PrismaticJointData pri = ref _prismaticJointsData[pIndex];
            Vec2 axis = Rot.Mul(_bodyRotations[joint.BodyC], pri.LocalAxisA);
            Vec2 rC = Rot.Mul(_bodyRotations[joint.BodyC], pri.LocalAnchorA - _bodyLocalCenters[joint.BodyC]);
            Vec2 rD = Rot.Mul(_bodyRotations[joint.BodyD], pri.LocalAnchorB - _bodyLocalCenters[joint.BodyD]);
            Vec2 d = (_bodyPositions[joint.BodyD] + rD) - (_bodyPositions[joint.BodyC] + rC);
            return Vec2.Dot(axis, d);
        }

        internal void InitGearJointVelocityConstraints(int index, float dt)
        {
            ref GearJointData joint = ref _gearJointsData[index];
            joint.JV_A = Vec2.Zero;
            joint.JV_B = Vec2.Zero;
            joint.JV_C = Vec2.Zero;
            joint.JV_D = Vec2.Zero;
            joint.JW_A = 0f;
            joint.JW_B = 0f;
            joint.JW_C = 0f;
            joint.JW_D = 0f;

            float mass = 0f;

            if (joint.TypeA == GearJointKind.Revolute)
            {
                joint.JW_A = -1f;
                joint.JW_B = 1f;
                mass += _bodyInverseInertias[joint.BodyA] + _bodyInverseInertias[joint.BodyB];
            }
            else
            {
                int pIndex = _prismaticJointIndexById[joint.JointAId];
                ref PrismaticJointData pri = ref _prismaticJointsData[pIndex];
                Vec2 axis = Rot.Mul(_bodyRotations[joint.BodyA], pri.LocalAxisA);
                Vec2 rA = Rot.Mul(_bodyRotations[joint.BodyA], pri.LocalAnchorA - _bodyLocalCenters[joint.BodyA]);
                Vec2 rB = Rot.Mul(_bodyRotations[joint.BodyB], pri.LocalAnchorB - _bodyLocalCenters[joint.BodyB]);
                Vec2 d = (_bodyPositions[joint.BodyB] + rB) - (_bodyPositions[joint.BodyA] + rA);
                float a1 = Vec2.Cross(d + rA, axis);
                float a2 = Vec2.Cross(rB, axis);

                joint.JV_A = -axis;
                joint.JW_A = -a1;
                joint.JV_B = axis;
                joint.JW_B = a2;
                mass += _bodyInverseMasses[joint.BodyA] + _bodyInverseMasses[joint.BodyB] +
                        _bodyInverseInertias[joint.BodyA] * a1 * a1 + _bodyInverseInertias[joint.BodyB] * a2 * a2;
            }

            if (joint.TypeB == GearJointKind.Revolute)
            {
                joint.JW_C = -joint.Ratio;
                joint.JW_D = joint.Ratio;
                mass += joint.Ratio * joint.Ratio * (_bodyInverseInertias[joint.BodyC] + _bodyInverseInertias[joint.BodyD]);
            }
            else
            {
                int pIndex = _prismaticJointIndexById[joint.JointBId];
                ref PrismaticJointData pri = ref _prismaticJointsData[pIndex];
                Vec2 axis = Rot.Mul(_bodyRotations[joint.BodyC], pri.LocalAxisA);
                Vec2 rC = Rot.Mul(_bodyRotations[joint.BodyC], pri.LocalAnchorA - _bodyLocalCenters[joint.BodyC]);
                Vec2 rD = Rot.Mul(_bodyRotations[joint.BodyD], pri.LocalAnchorB - _bodyLocalCenters[joint.BodyD]);
                Vec2 d = (_bodyPositions[joint.BodyD] + rD) - (_bodyPositions[joint.BodyC] + rC);
                float a1 = Vec2.Cross(d + rC, axis);
                float a2 = Vec2.Cross(rD, axis);

                joint.JV_C = -joint.Ratio * axis;
                joint.JW_C = -joint.Ratio * a1;
                joint.JV_D = joint.Ratio * axis;
                joint.JW_D = joint.Ratio * a2;
                mass += joint.Ratio * joint.Ratio * (_bodyInverseMasses[joint.BodyC] + _bodyInverseMasses[joint.BodyD] +
                        _bodyInverseInertias[joint.BodyC] * a1 * a1 + _bodyInverseInertias[joint.BodyD] * a2 * a2);
            }

            joint.Mass = mass > 0f ? 1f / mass : 0f;
        }

        internal void SolveGearJointVelocityConstraints(int index, float dt)
        {
            ref GearJointData joint = ref _gearJointsData[index];
            float Cdot = Vec2.Dot(joint.JV_A, _bodyLinearVelocities[joint.BodyA]) + joint.JW_A * _bodyAngularVelocities[joint.BodyA] +
                         Vec2.Dot(joint.JV_B, _bodyLinearVelocities[joint.BodyB]) + joint.JW_B * _bodyAngularVelocities[joint.BodyB] +
                         Vec2.Dot(joint.JV_C, _bodyLinearVelocities[joint.BodyC]) + joint.JW_C * _bodyAngularVelocities[joint.BodyC] +
                         Vec2.Dot(joint.JV_D, _bodyLinearVelocities[joint.BodyD]) + joint.JW_D * _bodyAngularVelocities[joint.BodyD];

            float impulse = -joint.Mass * Cdot;

            _bodyLinearVelocities[joint.BodyA] += _bodyInverseMasses[joint.BodyA] * impulse * joint.JV_A;
            _bodyAngularVelocities[joint.BodyA] += _bodyInverseInertias[joint.BodyA] * impulse * joint.JW_A;
            _bodyLinearVelocities[joint.BodyB] += _bodyInverseMasses[joint.BodyB] * impulse * joint.JV_B;
            _bodyAngularVelocities[joint.BodyB] += _bodyInverseInertias[joint.BodyB] * impulse * joint.JW_B;
            _bodyLinearVelocities[joint.BodyC] += _bodyInverseMasses[joint.BodyC] * impulse * joint.JV_C;
            _bodyAngularVelocities[joint.BodyC] += _bodyInverseInertias[joint.BodyC] * impulse * joint.JW_C;
            _bodyLinearVelocities[joint.BodyD] += _bodyInverseMasses[joint.BodyD] * impulse * joint.JV_D;
            _bodyAngularVelocities[joint.BodyD] += _bodyInverseInertias[joint.BodyD] * impulse * joint.JW_D;
        }

        internal void SolveGearJointPositionConstraints(int index)
        {
            ref GearJointData joint = ref _gearJointsData[index];
            float C = (GetGearCoordinateA(ref joint) + joint.Ratio * GetGearCoordinateB(ref joint)) - joint.Constant;
            float impulse = -joint.Mass * C;

            _bodyPositions[joint.BodyA] += _bodyInverseMasses[joint.BodyA] * impulse * joint.JV_A;
            _bodyPositions[joint.BodyB] += _bodyInverseMasses[joint.BodyB] * impulse * joint.JV_B;
            _bodyPositions[joint.BodyC] += _bodyInverseMasses[joint.BodyC] * impulse * joint.JV_C;
            _bodyPositions[joint.BodyD] += _bodyInverseMasses[joint.BodyD] * impulse * joint.JV_D;

            _bodyRotations[joint.BodyA] = new Rot(_bodyRotations[joint.BodyA].Angle + _bodyInverseInertias[joint.BodyA] * impulse * joint.JW_A);
            _bodyRotations[joint.BodyB] = new Rot(_bodyRotations[joint.BodyB].Angle + _bodyInverseInertias[joint.BodyB] * impulse * joint.JW_B);
            _bodyRotations[joint.BodyC] = new Rot(_bodyRotations[joint.BodyC].Angle + _bodyInverseInertias[joint.BodyC] * impulse * joint.JW_C);
            _bodyRotations[joint.BodyD] = new Rot(_bodyRotations[joint.BodyD].Angle + _bodyInverseInertias[joint.BodyD] * impulse * joint.JW_D);
        }
    }
}
