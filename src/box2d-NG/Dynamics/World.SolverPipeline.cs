namespace Box2DNG
{
    public sealed partial class World
    {
        private sealed class SolverPipeline
        {
            private readonly World _world;

            public SolverPipeline(World world)
            {
                _world = world;
            }

            public void Step(float timeStep)
            {
                if (timeStep <= 0f)
                {
                    return;
                }

                float dtRatio = _world._prevTimeStep > 0f ? timeStep / _world._prevTimeStep : 1f;
                _world._prevTimeStep = timeStep;

                _world.UpdateContacts();
                if (_world._islandsDirty || _world._lastIslands.Count == 0)
                {
                    _world.BuildIslands(awakeOnly: _world._def.EnableSleep);
                }

                IntegrateVelocities(timeStep);
                InitializeConstraints(timeStep, dtRatio);
                SolveVelocityConstraints(timeStep);
                _world.RaiseContactImpulseEvents();
                IntegratePositions(timeStep);
                SolvePositionConstraints(timeStep);
                FinalizeStep(timeStep);
            }

            private void IntegrateVelocities(float timeStep)
            {
                System.Collections.Generic.HashSet<Body> activeBodies = new System.Collections.Generic.HashSet<Body>();
                for (int i = 0; i < _world._awakeSet.Islands.Count; ++i)
                {
                    Island island = _world._awakeSet.Islands[i];
                    for (int j = 0; j < island.Bodies.Count; ++j)
                    {
                        activeBodies.Add(island.Bodies[j]);
                    }
                }

                for (int i = 0; i < _world._bodies.Count; ++i)
                {
                    Body body = _world._bodies[i];
                    if (body.Type == BodyType.Static)
                    {
                        continue;
                    }

                    if (_world._def.EnableSleep && body.AllowSleep == false)
                    {
                        body.SetAwake(true);
                    }

                    if (_world._def.EnableSleep && body.Awake == false)
                    {
                        continue;
                    }

                    if (activeBodies.Count > 0 && activeBodies.Contains(body) == false)
                    {
                        body.ClearForces();
                        continue;
                    }

                    if (body.Type == BodyType.Dynamic)
                    {
                        Vec2 accel = body.GravityScale * _world.Gravity;
                        if (body.InverseMass > 0f)
                        {
                            accel += body.InverseMass * body.Force;
                        }
                        body.LinearVelocity += timeStep * accel;
                        if (body.InverseInertia > 0f)
                        {
                            body.AngularVelocity += timeStep * body.InverseInertia * body.Torque;
                        }
                        body.LinearVelocity = ApplyLinearDamping(body.LinearVelocity, body.LinearDamping, timeStep);
                        body.AngularVelocity = ApplyAngularDamping(body.AngularVelocity, body.AngularDamping, timeStep);

                        if ((body.MotionLocks & MotionLocks.LinearX) != 0)
                        {
                            body.LinearVelocity = new Vec2(0f, body.LinearVelocity.Y);
                        }
                        if ((body.MotionLocks & MotionLocks.LinearY) != 0)
                        {
                            body.LinearVelocity = new Vec2(body.LinearVelocity.X, 0f);
                        }
                        if ((body.MotionLocks & MotionLocks.AngularZ) != 0)
                        {
                            body.AngularVelocity = 0f;
                        }
                    }

                    body.LinearVelocity = ClampLinearSpeed(body.LinearVelocity, _world._def.MaximumLinearSpeed);
                    body.AngularVelocity = ClampAngularSpeed(body.AngularVelocity, _world._def.MaximumAngularSpeed);
                    body.ClearForces();
                }
            }

            private void InitializeConstraints(float timeStep, float dtRatio)
            {
                _world.PrepareContacts(timeStep, dtRatio, _world._awakeSet.Contacts);

                for (int i = 0; i < _world._awakeSet.Joints.Count; ++i)
                {
                    JointHandle handle = _world._awakeSet.Joints[i];
                    InitJointVelocityConstraints(handle, timeStep);
                }
            }

            private void SolveVelocityConstraints(float timeStep)
            {
                _world.SolveContacts(timeStep, warmStart: true, _world._awakeSet.Contacts);

                for (int iter = 0; iter < _world._def.VelocityIterations; ++iter)
                {
                    _world.SolveContacts(timeStep, warmStart: false, _world._awakeSet.Contacts);
                    for (int i = 0; i < _world._awakeSet.Joints.Count; ++i)
                    {
                        JointHandle handle = _world._awakeSet.Joints[i];
                        SolveJointVelocityConstraints(handle, timeStep);
                    }
                }
            }

            private void IntegratePositions(float timeStep)
            {
                System.Collections.Generic.HashSet<Body> activeBodies = new System.Collections.Generic.HashSet<Body>();
                for (int i = 0; i < _world._awakeSet.Islands.Count; ++i)
                {
                    Island island = _world._awakeSet.Islands[i];
                    for (int j = 0; j < island.Bodies.Count; ++j)
                    {
                        activeBodies.Add(island.Bodies[j]);
                    }
                }

                for (int i = 0; i < _world._bodies.Count; ++i)
                {
                    Body body = _world._bodies[i];
                    if (body.Type == BodyType.Static)
                    {
                        continue;
                    }

                    if (_world._def.EnableSleep && body.Awake == false)
                    {
                        continue;
                    }

                    if (activeBodies.Count > 0 && activeBodies.Contains(body) == false)
                    {
                        continue;
                    }

                    Vec2 oldCenter = body.GetWorldCenter();
                    float oldAngle = body.Transform.Q.Angle;

                    Vec2 translation = timeStep * body.LinearVelocity;
                    float rotation = timeStep * body.AngularVelocity;

                    float maxTranslation = _world._def.MaximumTranslation;
                    float maxRotation = _world._def.MaximumRotation;
                    bool translationClamped = translation.LengthSquared > maxTranslation * maxTranslation;
                    bool rotationClamped = MathF.Abs(rotation) > maxRotation;

                    translation = ClampTranslation(translation, maxTranslation);
                    rotation = ClampRotation(rotation, maxRotation);
                    if (translationClamped && timeStep > 0f)
                    {
                        body.LinearVelocity = translation / timeStep;
                    }
                    if (rotationClamped && timeStep > 0f)
                    {
                        body.AngularVelocity = rotation / timeStep;
                    }

                    Vec2 newCenter = oldCenter + translation;
                    float newAngle = oldAngle + rotation;
                    Rot newRot = new Rot(newAngle);
                    Vec2 newPosition = newCenter - Rot.Mul(newRot, body.LocalCenter);
                    body.SetTransform(newPosition, newRot);

                    body.Sweep = new Sweep(body.LocalCenter, oldCenter, newCenter, oldAngle, newAngle, 0f);
                }
            }

            private void SolvePositionConstraints(float timeStep)
            {
                for (int iter = 0; iter < _world._def.PositionIterations; ++iter)
                {
                    _world.SolvePositionConstraints(_world._awakeSet.Contacts);
                    for (int i = 0; i < _world._awakeSet.Joints.Count; ++i)
                    {
                        JointHandle handle = _world._awakeSet.Joints[i];
                        SolveJointPositionConstraints(handle);
                    }
                }
            }

            private void FinalizeStep(float timeStep)
            {
                if (_world._def.EnableSleep)
                {
                    _world.UpdateSleep(timeStep);
                }
                _world.SolveTOI();
            }

            private void InitJointVelocityConstraints(JointHandle handle, float timeStep)
            {
                switch (handle.Type)
                {
                    case JointType.Distance:
                        _world._distanceJoints[handle.Index].InitVelocityConstraints(timeStep);
                        break;
                    case JointType.Revolute:
                        _world._revoluteJoints[handle.Index].InitVelocityConstraints(timeStep);
                        break;
                    case JointType.Prismatic:
                        _world._prismaticJoints[handle.Index].InitVelocityConstraints(timeStep);
                        break;
                    case JointType.Wheel:
                        _world._wheelJoints[handle.Index].InitVelocityConstraints(timeStep);
                        break;
                    case JointType.Pulley:
                        _world._pulleyJoints[handle.Index].InitVelocityConstraints(timeStep);
                        break;
                    case JointType.Weld:
                        _world._weldJoints[handle.Index].InitVelocityConstraints(timeStep);
                        break;
                    case JointType.Motor:
                        _world._motorJoints[handle.Index].InitVelocityConstraints(timeStep);
                        break;
                    case JointType.Gear:
                        _world._gearJoints[handle.Index].InitVelocityConstraints(timeStep);
                        break;
                    case JointType.Rope:
                        _world._ropeJoints[handle.Index].InitVelocityConstraints(timeStep);
                        break;
                    case JointType.Friction:
                        _world._frictionJoints[handle.Index].InitVelocityConstraints(timeStep);
                        break;
                }
            }

            private void SolveJointVelocityConstraints(JointHandle handle, float timeStep)
            {
                switch (handle.Type)
                {
                    case JointType.Distance:
                        _world._distanceJoints[handle.Index].SolveVelocityConstraints();
                        break;
                    case JointType.Revolute:
                        _world._revoluteJoints[handle.Index].SolveVelocityConstraints(timeStep);
                        break;
                    case JointType.Prismatic:
                        _world._prismaticJoints[handle.Index].SolveVelocityConstraints(timeStep);
                        break;
                    case JointType.Wheel:
                        _world._wheelJoints[handle.Index].SolveVelocityConstraints(timeStep);
                        break;
                    case JointType.Pulley:
                        _world._pulleyJoints[handle.Index].SolveVelocityConstraints();
                        break;
                    case JointType.Weld:
                        _world._weldJoints[handle.Index].SolveVelocityConstraints(timeStep);
                        break;
                    case JointType.Motor:
                        _world._motorJoints[handle.Index].SolveVelocityConstraints(timeStep);
                        break;
                    case JointType.Gear:
                        _world._gearJoints[handle.Index].SolveVelocityConstraints(timeStep);
                        break;
                    case JointType.Rope:
                        _world._ropeJoints[handle.Index].SolveVelocityConstraints(timeStep);
                        break;
                    case JointType.Friction:
                        _world._frictionJoints[handle.Index].SolveVelocityConstraints(timeStep);
                        break;
                }
            }

            private void SolveJointPositionConstraints(JointHandle handle)
            {
                switch (handle.Type)
                {
                    case JointType.Distance:
                        _world._distanceJoints[handle.Index].SolvePositionConstraints();
                        break;
                    case JointType.Revolute:
                        _world._revoluteJoints[handle.Index].SolvePositionConstraints();
                        break;
                    case JointType.Prismatic:
                        _world._prismaticJoints[handle.Index].SolvePositionConstraints();
                        break;
                    case JointType.Wheel:
                        _world._wheelJoints[handle.Index].SolvePositionConstraints();
                        break;
                    case JointType.Pulley:
                        _world._pulleyJoints[handle.Index].SolvePositionConstraints();
                        break;
                    case JointType.Weld:
                        _world._weldJoints[handle.Index].SolvePositionConstraints();
                        break;
                    case JointType.Motor:
                        _world._motorJoints[handle.Index].SolvePositionConstraints();
                        break;
                    case JointType.Gear:
                        _world._gearJoints[handle.Index].SolvePositionConstraints();
                        break;
                    case JointType.Rope:
                        _world._ropeJoints[handle.Index].SolvePositionConstraints();
                        break;
                    case JointType.Friction:
                        _world._frictionJoints[handle.Index].SolvePositionConstraints();
                        break;
                }
            }
        }
    }
}
