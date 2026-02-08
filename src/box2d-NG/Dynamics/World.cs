using System;

namespace Box2DNG
{
    public sealed class World
    {
        private readonly WorldDef _def;
        private readonly WorldEvents _events = new WorldEvents();
        private readonly BroadPhase<Fixture> _broadPhase = new BroadPhase<Fixture>();
        private readonly System.Collections.Generic.List<Body> _bodies = new System.Collections.Generic.List<Body>();
        private readonly System.Collections.Generic.List<Contact> _contacts = new System.Collections.Generic.List<Contact>();
        private readonly System.Collections.Generic.Dictionary<ContactKey, Contact> _contactMap = new System.Collections.Generic.Dictionary<ContactKey, Contact>();
        private readonly System.Collections.Generic.HashSet<ContactKey> _sensorPairs = new System.Collections.Generic.HashSet<ContactKey>();
        private readonly System.Collections.Generic.Dictionary<ContactKey, (object? A, object? B)> _sensorUserData = new System.Collections.Generic.Dictionary<ContactKey, (object? A, object? B)>();
        private readonly System.Collections.Generic.List<DistanceJoint> _distanceJoints = new System.Collections.Generic.List<DistanceJoint>();
        private readonly System.Collections.Generic.List<RevoluteJoint> _revoluteJoints = new System.Collections.Generic.List<RevoluteJoint>();
        private readonly System.Collections.Generic.List<PrismaticJoint> _prismaticJoints = new System.Collections.Generic.List<PrismaticJoint>();
        private readonly System.Collections.Generic.List<WheelJoint> _wheelJoints = new System.Collections.Generic.List<WheelJoint>();
        private readonly System.Collections.Generic.List<PulleyJoint> _pulleyJoints = new System.Collections.Generic.List<PulleyJoint>();
        private readonly System.Collections.Generic.List<WeldJoint> _weldJoints = new System.Collections.Generic.List<WeldJoint>();
        private readonly System.Collections.Generic.List<MotorJoint> _motorJoints = new System.Collections.Generic.List<MotorJoint>();
        private readonly System.Collections.Generic.List<GearJoint> _gearJoints = new System.Collections.Generic.List<GearJoint>();
        private readonly System.Collections.Generic.List<RopeJoint> _ropeJoints = new System.Collections.Generic.List<RopeJoint>();
        private readonly System.Collections.Generic.List<FrictionJoint> _frictionJoints = new System.Collections.Generic.List<FrictionJoint>();
        private float _prevTimeStep;

        public World(WorldDef def)
        {
            _def = def ?? throw new ArgumentNullException(nameof(def));
        }

        public Vec2 Gravity => _def.Gravity;

        public WorldEvents Events => _events;

        public System.Collections.Generic.IReadOnlyList<Body> Bodies => _bodies;

        public System.Collections.Generic.IReadOnlyList<Contact> Contacts => _contacts;
        public System.Collections.Generic.IReadOnlyList<DistanceJoint> DistanceJoints => _distanceJoints;
        public System.Collections.Generic.IReadOnlyList<RevoluteJoint> RevoluteJoints => _revoluteJoints;
        public System.Collections.Generic.IReadOnlyList<PrismaticJoint> PrismaticJoints => _prismaticJoints;
        public System.Collections.Generic.IReadOnlyList<WheelJoint> WheelJoints => _wheelJoints;
        public System.Collections.Generic.IReadOnlyList<PulleyJoint> PulleyJoints => _pulleyJoints;
        public System.Collections.Generic.IReadOnlyList<WeldJoint> WeldJoints => _weldJoints;
        public System.Collections.Generic.IReadOnlyList<MotorJoint> MotorJoints => _motorJoints;
        public System.Collections.Generic.IReadOnlyList<GearJoint> GearJoints => _gearJoints;
        public System.Collections.Generic.IReadOnlyList<RopeJoint> RopeJoints => _ropeJoints;
        public System.Collections.Generic.IReadOnlyList<FrictionJoint> FrictionJoints => _frictionJoints;

        public Body CreateBody(BodyDef def)
        {
            Body body = new Body(this, def);
            _bodies.Add(body);
            return body;
        }

        public DistanceJoint CreateJoint(DistanceJointDef def)
        {
            DistanceJoint joint = new DistanceJoint(def);
            _distanceJoints.Add(joint);
            return joint;
        }

        public RevoluteJoint CreateJoint(RevoluteJointDef def)
        {
            RevoluteJoint joint = new RevoluteJoint(def);
            _revoluteJoints.Add(joint);
            return joint;
        }

        public PrismaticJoint CreateJoint(PrismaticJointDef def)
        {
            PrismaticJoint joint = new PrismaticJoint(def);
            _prismaticJoints.Add(joint);
            return joint;
        }

        public WheelJoint CreateJoint(WheelJointDef def)
        {
            WheelJoint joint = new WheelJoint(def);
            _wheelJoints.Add(joint);
            return joint;
        }

        public PulleyJoint CreateJoint(PulleyJointDef def)
        {
            PulleyJoint joint = new PulleyJoint(def);
            _pulleyJoints.Add(joint);
            return joint;
        }

        public WeldJoint CreateJoint(WeldJointDef def)
        {
            WeldJoint joint = new WeldJoint(def);
            _weldJoints.Add(joint);
            return joint;
        }

        public MotorJoint CreateJoint(MotorJointDef def)
        {
            MotorJoint joint = new MotorJoint(def);
            _motorJoints.Add(joint);
            return joint;
        }

        public GearJoint CreateJoint(GearJointDef def)
        {
            GearJoint joint = new GearJoint(def);
            _gearJoints.Add(joint);
            return joint;
        }

        public RopeJoint CreateJoint(RopeJointDef def)
        {
            RopeJoint joint = new RopeJoint(def);
            _ropeJoints.Add(joint);
            return joint;
        }

        public FrictionJoint CreateJoint(FrictionJointDef def)
        {
            FrictionJoint joint = new FrictionJoint(def);
            _frictionJoints.Add(joint);
            return joint;
        }

        internal Fixture CreateFixture(Body body, Shape shape)
        {
            return CreateFixture(body, new FixtureDef(shape));
        }

        internal Fixture CreateFixture(Body body, FixtureDef def)
        {
            Fixture fixture = new Fixture(body, def.Shape)
            {
                Friction = def.Friction,
                Restitution = def.Restitution,
                Density = def.Density,
                IsSensor = def.IsSensor,
                Filter = def.Filter,
                UserData = def.UserData
            };
            fixture.Aabb = ShapeGeometry.ComputeAabb(def.Shape, body.Transform);
            fixture.ProxyId = _broadPhase.CreateProxy(fixture.Aabb, fixture);
            return fixture;
        }

        internal void DestroyFixture(Fixture fixture)
        {
            if (fixture.ProxyId >= 0)
            {
                _broadPhase.DestroyProxy(fixture.ProxyId);
            }

            for (int i = _contacts.Count - 1; i >= 0; --i)
            {
                Contact contact = _contacts[i];
                if (contact.FixtureA == fixture || contact.FixtureB == fixture)
                {
                    _contacts.RemoveAt(i);
                }
            }

            System.Collections.Generic.List<ContactKey> remove = new System.Collections.Generic.List<ContactKey>();
            foreach (var pair in _contactMap)
            {
                Contact contact = pair.Value;
                if (contact.FixtureA == fixture || contact.FixtureB == fixture)
                {
                    remove.Add(pair.Key);
                }
            }
            for (int i = 0; i < remove.Count; ++i)
            {
                _contactMap.Remove(remove[i]);
            }

            if (fixture.ProxyId >= 0 && _sensorPairs.Count > 0)
            {
                System.Collections.Generic.List<ContactKey> sensorRemove = new System.Collections.Generic.List<ContactKey>();
                foreach (var key in _sensorPairs)
                {
                    if (key.A == fixture.ProxyId || key.B == fixture.ProxyId)
                    {
                        sensorRemove.Add(key);
                    }
                }
                for (int i = 0; i < sensorRemove.Count; ++i)
                {
                    _sensorPairs.Remove(sensorRemove[i]);
                    _sensorUserData.Remove(sensorRemove[i]);
                }
            }
        }

        public Contact CreateContact(Shape shapeA, Transform transformA, Shape shapeB, Transform transformB)
        {
            Contact contact = new Contact(shapeA, transformA, shapeB, transformB);
            contact.Evaluate();
            return contact;
        }

        public void QueryAabb(Aabb aabb, System.Func<Fixture, bool> callback, QueryFilter? filter = null, bool includeSensors = true)
        {
            QueryFilter useFilter = filter ?? QueryFilter.Default;
            _broadPhase.Query(proxyId =>
            {
                Fixture? fixture = _broadPhase.GetUserData(proxyId);
                if (fixture == null)
                {
                    return true;
                }
                if (!includeSensors && fixture.IsSensor)
                {
                    return true;
                }
                if (!PassesQueryFilter(fixture.Filter, useFilter))
                {
                    return true;
                }
                return callback(fixture);
            }, aabb);
        }

        public System.Collections.Generic.List<Fixture> QueryAabb(Aabb aabb, QueryFilter? filter = null, bool includeSensors = true)
        {
            System.Collections.Generic.List<Fixture> results = new System.Collections.Generic.List<Fixture>();
            QueryAabb(aabb, fixture =>
            {
                results.Add(fixture);
                return true;
            }, filter, includeSensors);
            return results;
        }

        public bool RayCast(RayCastInput input, out RayCastHit hit, QueryFilter? filter = null, bool includeSensors = true)
        {
            if (!Collision.IsValidRay(input))
            {
                hit = default;
                return false;
            }

            QueryFilter useFilter = filter ?? QueryFilter.Default;
            float closest = input.MaxFraction;
            bool found = false;
            RayCastHit bestHit = default;

            _broadPhase.RayCast(proxyId =>
            {
                Fixture? fixture = _broadPhase.GetUserData(proxyId);
                if (fixture == null)
                {
                    return closest;
                }
                if (!includeSensors && fixture.IsSensor)
                {
                    return closest;
                }
                if (!PassesQueryFilter(fixture.Filter, useFilter))
                {
                    return closest;
                }

                if (!TryRayCastFixture(fixture, input, out RayCastOutput output))
                {
                    return closest;
                }

                if (output.Fraction < closest)
                {
                    closest = output.Fraction;
                    found = true;
                    bestHit = new RayCastHit(fixture, output.Point, output.Normal, output.Fraction);
                }

                return closest;
            }, input);

            hit = bestHit;
            return found;
        }

        public System.Collections.Generic.List<RayCastHit> RayCastAll(RayCastInput input, QueryFilter? filter = null, bool includeSensors = true)
        {
            System.Collections.Generic.List<RayCastHit> hits = new System.Collections.Generic.List<RayCastHit>();
            if (!Collision.IsValidRay(input))
            {
                return hits;
            }

            QueryFilter useFilter = filter ?? QueryFilter.Default;
            _broadPhase.RayCast(proxyId =>
            {
                Fixture? fixture = _broadPhase.GetUserData(proxyId);
                if (fixture == null)
                {
                    return input.MaxFraction;
                }
                if (!includeSensors && fixture.IsSensor)
                {
                    return input.MaxFraction;
                }
                if (!PassesQueryFilter(fixture.Filter, useFilter))
                {
                    return input.MaxFraction;
                }
                if (!TryRayCastFixture(fixture, input, out RayCastOutput output))
                {
                    return input.MaxFraction;
                }

                hits.Add(new RayCastHit(fixture, output.Point, output.Normal, output.Fraction));
                return input.MaxFraction;
            }, input);

            hits.Sort((a, b) => a.Fraction.CompareTo(b.Fraction));
            return hits;
        }

        public void UpdateContacts()
        {
            for (int i = 0; i < _bodies.Count; ++i)
            {
                Body body = _bodies[i];
                for (int j = 0; j < body.Fixtures.Count; ++j)
                {
                    Fixture fixture = body.Fixtures[j];
                    Aabb newAabb = ShapeGeometry.ComputeAabb(fixture.Shape, body.Transform);
                    Vec2 oldCenter = fixture.Aabb.Center;
                    Vec2 newCenter = newAabb.Center;
                    Vec2 displacement = newCenter - oldCenter;
                    _broadPhase.MoveProxy(fixture.ProxyId, newAabb, displacement);
                    _broadPhase.TouchProxy(fixture.ProxyId);
                    fixture.Aabb = newAabb;
                }
            }

            _contacts.Clear();
            System.Collections.Generic.HashSet<ContactKey> seen = new System.Collections.Generic.HashSet<ContactKey>();
            System.Collections.Generic.HashSet<ContactKey> sensorSeen = new System.Collections.Generic.HashSet<ContactKey>();

            System.Collections.Generic.List<ContactBeginEvent> beginEvents = new System.Collections.Generic.List<ContactBeginEvent>();
            System.Collections.Generic.List<ContactEndEvent> endEvents = new System.Collections.Generic.List<ContactEndEvent>();
            System.Collections.Generic.List<ContactHitEvent> hitEvents = new System.Collections.Generic.List<ContactHitEvent>();
            System.Collections.Generic.List<SensorBeginEvent> sensorBeginEvents = new System.Collections.Generic.List<SensorBeginEvent>();
            System.Collections.Generic.List<SensorEndEvent> sensorEndEvents = new System.Collections.Generic.List<SensorEndEvent>();

            _broadPhase.UpdatePairs((fixtureA, fixtureB) =>
            {
                if (fixtureA == null || fixtureB == null)
                {
                    return;
                }

                if (fixtureA.IsSensor || fixtureB.IsSensor)
                {
                    if (TestOverlap(fixtureA, fixtureB))
                    {
                        ContactKey sensorKey = new ContactKey(fixtureA.ProxyId, fixtureB.ProxyId);
                        sensorSeen.Add(sensorKey);
                        if (!_sensorPairs.Contains(sensorKey))
                        {
                            sensorBeginEvents.Add(new SensorBeginEvent(fixtureA.UserData, fixtureB.UserData));
                            _sensorUserData[sensorKey] = (fixtureA.UserData, fixtureB.UserData);
                        }
                    }
                    return;
                }

                if (!ShouldCollide(fixtureA, fixtureB))
                {
                    return;
                }

                ContactKey contactKey = new ContactKey(fixtureA.ProxyId, fixtureB.ProxyId);
                seen.Add(contactKey);

                if (!_contactMap.TryGetValue(contactKey, out Contact? contact))
                {
                    contact = new Contact(fixtureA, fixtureB);
                    contact.Evaluate();
                    _contactMap[contactKey] = contact;
                }
                else
                {
                    contact.Update(fixtureA.Body.Transform, fixtureB.Body.Transform);
                }

                if (contact.IsTouching)
                {
                    Body bodyA = fixtureA.Body;
                    Body bodyB = fixtureB.Body;
                    if (bodyA.Type != BodyType.Static && bodyB.Type != BodyType.Static)
                    {
                        if (bodyA.Type == BodyType.Kinematic || bodyB.Type == BodyType.Kinematic || bodyA.Awake || bodyB.Awake)
                        {
                            bodyA.SetAwake(true);
                            bodyB.SetAwake(true);
                        }
                    }
                }

                if (contact.IsTouching && !contact.WasTouching)
                {
                    beginEvents.Add(new ContactBeginEvent(fixtureA.UserData, fixtureB.UserData));
                }
                else if (!contact.IsTouching && contact.WasTouching)
                {
                    endEvents.Add(new ContactEndEvent(fixtureA.UserData, fixtureB.UserData));
                }

                if (contact.Manifold.PointCount > 0)
                {
                    _contacts.Add(contact);
                    AddHitEvent(contact, hitEvents);
                }
            });

            RemoveStaleContacts(seen, endEvents);
            RemoveStaleSensors(sensorSeen, sensorEndEvents);

            if (beginEvents.Count > 0 || endEvents.Count > 0 || hitEvents.Count > 0)
            {
                _events.Raise(new ContactEvents(beginEvents.ToArray(), endEvents.ToArray(), hitEvents.ToArray()));
            }
            if (sensorBeginEvents.Count > 0 || sensorEndEvents.Count > 0)
            {
                _events.Raise(new SensorEvents(sensorBeginEvents.ToArray(), sensorEndEvents.ToArray()));
            }
        }

        public void Step(float timeStep)
        {
            if (timeStep <= 0f)
            {
                return;
            }

            float dtRatio = _prevTimeStep > 0f ? timeStep / _prevTimeStep : 1f;
            _prevTimeStep = timeStep;

            for (int i = 0; i < _bodies.Count; ++i)
            {
                Body body = _bodies[i];
                if (body.Type == BodyType.Static)
                {
                    continue;
                }

                if (_def.EnableSleep && body.AllowSleep == false)
                {
                    body.SetAwake(true);
                }

                if (_def.EnableSleep && body.Awake == false)
                {
                    continue;
                }

                if (body.Type == BodyType.Dynamic)
                {
                    Vec2 accel = body.GravityScale * Gravity;
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

                body.LinearVelocity = ClampLinearSpeed(body.LinearVelocity, _def.MaximumLinearSpeed);
                body.AngularVelocity = ClampAngularSpeed(body.AngularVelocity, _def.MaximumAngularSpeed);
                body.ClearForces();
            }

            UpdateContacts();
            PrepareContacts(timeStep, dtRatio);
            for (int i = 0; i < _distanceJoints.Count; ++i)
            {
                _distanceJoints[i].InitVelocityConstraints(timeStep);
            }
            for (int i = 0; i < _revoluteJoints.Count; ++i)
            {
                _revoluteJoints[i].InitVelocityConstraints(timeStep);
            }
            for (int i = 0; i < _prismaticJoints.Count; ++i)
            {
                _prismaticJoints[i].InitVelocityConstraints(timeStep);
            }
            for (int i = 0; i < _wheelJoints.Count; ++i)
            {
                _wheelJoints[i].InitVelocityConstraints(timeStep);
            }
            for (int i = 0; i < _pulleyJoints.Count; ++i)
            {
                _pulleyJoints[i].InitVelocityConstraints(timeStep);
            }
            for (int i = 0; i < _weldJoints.Count; ++i)
            {
                _weldJoints[i].InitVelocityConstraints(timeStep);
            }
            for (int i = 0; i < _motorJoints.Count; ++i)
            {
                _motorJoints[i].InitVelocityConstraints(timeStep);
            }
            for (int i = 0; i < _gearJoints.Count; ++i)
            {
                _gearJoints[i].InitVelocityConstraints(timeStep);
            }
            for (int i = 0; i < _ropeJoints.Count; ++i)
            {
                _ropeJoints[i].InitVelocityConstraints(timeStep);
            }
            for (int i = 0; i < _frictionJoints.Count; ++i)
            {
                _frictionJoints[i].InitVelocityConstraints(timeStep);
            }
            SolveContacts(timeStep, warmStart: true);
            for (int i = 0; i < _def.VelocityIterations; ++i)
            {
                SolveContacts(timeStep, warmStart: false);
                for (int j = 0; j < _distanceJoints.Count; ++j)
                {
                    _distanceJoints[j].SolveVelocityConstraints();
                }
                for (int j = 0; j < _revoluteJoints.Count; ++j)
                {
                    _revoluteJoints[j].SolveVelocityConstraints(timeStep);
                }
                for (int j = 0; j < _prismaticJoints.Count; ++j)
                {
                    _prismaticJoints[j].SolveVelocityConstraints(timeStep);
                }
                for (int j = 0; j < _wheelJoints.Count; ++j)
                {
                    _wheelJoints[j].SolveVelocityConstraints(timeStep);
                }
                for (int j = 0; j < _pulleyJoints.Count; ++j)
                {
                    _pulleyJoints[j].SolveVelocityConstraints();
                }
                for (int j = 0; j < _weldJoints.Count; ++j)
                {
                    _weldJoints[j].SolveVelocityConstraints(timeStep);
                }
                for (int j = 0; j < _motorJoints.Count; ++j)
                {
                    _motorJoints[j].SolveVelocityConstraints(timeStep);
                }
                for (int j = 0; j < _gearJoints.Count; ++j)
                {
                    _gearJoints[j].SolveVelocityConstraints(timeStep);
                }
                for (int j = 0; j < _ropeJoints.Count; ++j)
                {
                    _ropeJoints[j].SolveVelocityConstraints(timeStep);
                }
                for (int j = 0; j < _frictionJoints.Count; ++j)
                {
                    _frictionJoints[j].SolveVelocityConstraints(timeStep);
                }
            }

            RaiseContactImpulseEvents();

            for (int i = 0; i < _bodies.Count; ++i)
            {
                Body body = _bodies[i];
                if (body.Type == BodyType.Static)
                {
                    continue;
                }

                if (_def.EnableSleep && body.Awake == false)
                {
                    continue;
                }

                Vec2 oldCenter = body.GetWorldCenter();
                float oldAngle = body.Transform.Q.Angle;

                Vec2 translation = timeStep * body.LinearVelocity;
                float rotation = timeStep * body.AngularVelocity;

                float maxTranslation = _def.MaximumTranslation;
                float maxRotation = _def.MaximumRotation;
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

            for (int i = 0; i < _def.PositionIterations; ++i)
            {
                SolvePositionConstraints();
                for (int j = 0; j < _distanceJoints.Count; ++j)
                {
                    _distanceJoints[j].SolvePositionConstraints();
                }
                for (int j = 0; j < _revoluteJoints.Count; ++j)
                {
                    _revoluteJoints[j].SolvePositionConstraints();
                }
                for (int j = 0; j < _prismaticJoints.Count; ++j)
                {
                    _prismaticJoints[j].SolvePositionConstraints();
                }
                for (int j = 0; j < _wheelJoints.Count; ++j)
                {
                    _wheelJoints[j].SolvePositionConstraints();
                }
                for (int j = 0; j < _pulleyJoints.Count; ++j)
                {
                    _pulleyJoints[j].SolvePositionConstraints();
                }
                for (int j = 0; j < _weldJoints.Count; ++j)
                {
                    _weldJoints[j].SolvePositionConstraints();
                }
                for (int j = 0; j < _motorJoints.Count; ++j)
                {
                    _motorJoints[j].SolvePositionConstraints();
                }
                for (int j = 0; j < _gearJoints.Count; ++j)
                {
                    _gearJoints[j].SolvePositionConstraints();
                }
                for (int j = 0; j < _ropeJoints.Count; ++j)
                {
                    _ropeJoints[j].SolvePositionConstraints();
                }
                for (int j = 0; j < _frictionJoints.Count; ++j)
                {
                    _frictionJoints[j].SolvePositionConstraints();
                }
            }
            if (_def.EnableSleep)
            {
                UpdateSleep(timeStep);
            }
            SolveTOI();
        }

        private void RaiseContactImpulseEvents()
        {
            if (_contacts.Count == 0)
            {
                return;
            }

            System.Collections.Generic.List<ContactImpulseEvent> impulseEvents = new System.Collections.Generic.List<ContactImpulseEvent>();
            for (int i = 0; i < _contacts.Count; ++i)
            {
                Contact contact = _contacts[i];
                if (contact.FixtureA == null || contact.FixtureB == null)
                {
                    continue;
                }

                if (contact.Manifold.PointCount == 0)
                {
                    continue;
                }

                Fixture fixtureA = contact.FixtureA;
                Fixture fixtureB = contact.FixtureB;
                Body bodyA = fixtureA.Body;
                Body bodyB = fixtureB.Body;

                float radiusA = GetShapeRadius(fixtureA.Shape);
                float radiusB = GetShapeRadius(fixtureB.Shape);
                WorldManifold worldManifold = new WorldManifold();
                worldManifold.Initialize(contact.Manifold, bodyA.Transform, radiusA, bodyB.Transform, radiusB);

                Vec2 normal = worldManifold.Normal;
                for (int p = 0; p < contact.Manifold.PointCount; ++p)
                {
                    ManifoldPoint mp = contact.Manifold.Points[p];
                    Vec2 point = worldManifold.Points[p];
                    impulseEvents.Add(new ContactImpulseEvent(
                        fixtureA.UserData,
                        fixtureB.UserData,
                        point,
                        normal,
                        mp.NormalImpulse,
                        mp.TangentImpulse));
                }
            }

            if (impulseEvents.Count > 0)
            {
                _events.Raise(new ContactImpulseEvents(impulseEvents.ToArray()));
            }
        }

        private void SolveContacts(float timeStep, bool warmStart)
        {
            for (int i = 0; i < _contacts.Count; ++i)
            {
                Contact contact = _contacts[i];
                if (contact.FixtureA == null || contact.FixtureB == null)
                {
                    continue;
                }

                Fixture fixtureA = contact.FixtureA;
                Fixture fixtureB = contact.FixtureB;
                Body bodyA = fixtureA.Body;
                Body bodyB = fixtureB.Body;

                if (bodyA.Type == BodyType.Static && bodyB.Type == BodyType.Static)
                {
                    continue;
                }

                float radiusA = GetShapeRadius(fixtureA.Shape);
                float radiusB = GetShapeRadius(fixtureB.Shape);

                WorldManifold worldManifold = new WorldManifold();
                worldManifold.Initialize(contact.Manifold, bodyA.Transform, radiusA, bodyB.Transform, radiusB);

                Vec2 normal = worldManifold.Normal;
                Vec2 tangent = new Vec2(-normal.Y, normal.X);

                float friction = MixFriction(fixtureA, fixtureB);
                float invMassA = bodyA.InverseMass;
                float invMassB = bodyB.InverseMass;
                float invIA = bodyA.InverseInertia;
                float invIB = bodyB.InverseInertia;
                int pointCount = contact.Manifold.PointCount;
                int solvePointCount = pointCount;
                int primaryIndex = 0;
                bool useBlockSolver = false;
                float k11 = 0f;
                float k22 = 0f;
                float k12 = 0f;
                float det = 0f;

                if (pointCount == 2)
                {
                    SolverPoint sp1 = contact.SolverPoints[0];
                    SolverPoint sp2 = contact.SolverPoints[1];

                    Vec2 rA1 = sp1.RA;
                    Vec2 rB1 = sp1.RB;
                    Vec2 rA2 = sp2.RA;
                    Vec2 rB2 = sp2.RB;

                    float rnA1 = Vec2.Cross(rA1, normal);
                    float rnB1 = Vec2.Cross(rB1, normal);
                    float rnA2 = Vec2.Cross(rA2, normal);
                    float rnB2 = Vec2.Cross(rB2, normal);

                    k11 = invMassA + invMassB + invIA * rnA1 * rnA1 + invIB * rnB1 * rnB1;
                    k22 = invMassA + invMassB + invIA * rnA2 * rnA2 + invIB * rnB2 * rnB2;
                    k12 = invMassA + invMassB + invIA * rnA1 * rnA2 + invIB * rnB1 * rnB2;

                    det = k11 * k22 - k12 * k12;
                    const float kMaxConditionNumber = 1000f;
                    if (det > Constants.Epsilon && k11 * k11 < kMaxConditionNumber * det)
                    {
                        useBlockSolver = true;
                    }
                    else
                    {
                        solvePointCount = 1;
                        PositionManifold pm0 = ComputePositionManifold(contact, bodyA.Transform, bodyB.Transform, radiusA, radiusB, 0);
                        PositionManifold pm1 = ComputePositionManifold(contact, bodyA.Transform, bodyB.Transform, radiusA, radiusB, 1);
                        if (pm1.Separation < pm0.Separation)
                        {
                            primaryIndex = 1;
                        }

                        int otherIndex = 1 - primaryIndex;
                        ManifoldPoint otherMp = contact.Manifold.Points[otherIndex];
                        if (otherMp.NormalImpulse != 0f || otherMp.TangentImpulse != 0f)
                        {
                            contact.Manifold.Points[otherIndex] = new ManifoldPoint(otherMp.LocalPoint, 0f, 0f, otherMp.Id);
                        }
                    }
                }

                // Warm start
                if (warmStart)
                {
                    if (solvePointCount == 1)
                    {
                        int index = pointCount == 2 ? primaryIndex : 0;
                        SolverPoint sp = contact.SolverPoints[index];
                        Vec2 rA = sp.RA;
                        Vec2 rB = sp.RB;
                        ManifoldPoint mp = contact.Manifold.Points[index];
                        if (mp.NormalImpulse != 0f || mp.TangentImpulse != 0f)
                        {
                            Vec2 warm = mp.NormalImpulse * normal + mp.TangentImpulse * tangent;
                            bodyA.LinearVelocity -= invMassA * warm;
                            bodyB.LinearVelocity += invMassB * warm;
                            bodyA.AngularVelocity -= invIA * Vec2.Cross(rA, warm);
                            bodyB.AngularVelocity += invIB * Vec2.Cross(rB, warm);
                        }
                    }
                    else
                    {
                        for (int p = 0; p < pointCount; ++p)
                        {
                            SolverPoint sp = contact.SolverPoints[p];
                            Vec2 rA = sp.RA;
                            Vec2 rB = sp.RB;
                            ManifoldPoint mp = contact.Manifold.Points[p];
                            if (mp.NormalImpulse == 0f && mp.TangentImpulse == 0f)
                            {
                                continue;
                            }

                            Vec2 warm = mp.NormalImpulse * normal + mp.TangentImpulse * tangent;
                            bodyA.LinearVelocity -= invMassA * warm;
                            bodyB.LinearVelocity += invMassB * warm;
                            bodyA.AngularVelocity -= invIA * Vec2.Cross(rA, warm);
                            bodyB.AngularVelocity += invIB * Vec2.Cross(rB, warm);
                        }
                    }
                }

                // Friction constraints
                if (solvePointCount == 1)
                {
                    int index = pointCount == 2 ? primaryIndex : 0;
                    SolverPoint sp = contact.SolverPoints[index];
                    Vec2 rA = sp.RA;
                    Vec2 rB = sp.RB;
                    ManifoldPoint mp = contact.Manifold.Points[index];

                    Vec2 vA = bodyA.LinearVelocity + Vec2.Cross(bodyA.AngularVelocity, rA);
                    Vec2 vB = bodyB.LinearVelocity + Vec2.Cross(bodyB.AngularVelocity, rB);
                    Vec2 dv = vB - vA;
                    float vt = Vec2.Dot(dv, tangent);

                    if (sp.TangentMass > 0f)
                    {
                        float tangentImpulse = -vt * sp.TangentMass;
                        float maxFriction = friction * mp.NormalImpulse;
                        float newTangentImpulse = MathFng.Clamp(mp.TangentImpulse + tangentImpulse, -maxFriction, maxFriction);
                        float appliedTangent = newTangentImpulse - mp.TangentImpulse;
                        Vec2 Pt = appliedTangent * tangent;

                        bodyA.LinearVelocity -= invMassA * Pt;
                        bodyB.LinearVelocity += invMassB * Pt;
                        bodyA.AngularVelocity -= invIA * Vec2.Cross(rA, Pt);
                        bodyB.AngularVelocity += invIB * Vec2.Cross(rB, Pt);

                        mp = new ManifoldPoint(mp.LocalPoint, mp.NormalImpulse, newTangentImpulse, mp.Id);
                        contact.Manifold.Points[index] = mp;
                    }
                }
                else
                {
                    for (int p = 0; p < pointCount; ++p)
                    {
                        SolverPoint sp = contact.SolverPoints[p];
                        Vec2 rA = sp.RA;
                        Vec2 rB = sp.RB;
                        ManifoldPoint mp = contact.Manifold.Points[p];

                        Vec2 vA = bodyA.LinearVelocity + Vec2.Cross(bodyA.AngularVelocity, rA);
                        Vec2 vB = bodyB.LinearVelocity + Vec2.Cross(bodyB.AngularVelocity, rB);
                        Vec2 dv = vB - vA;
                        float vt = Vec2.Dot(dv, tangent);

                        if (sp.TangentMass > 0f)
                        {
                            float tangentImpulse = -vt * sp.TangentMass;
                            float maxFriction = friction * mp.NormalImpulse;
                            float newTangentImpulse = MathFng.Clamp(mp.TangentImpulse + tangentImpulse, -maxFriction, maxFriction);
                            float appliedTangent = newTangentImpulse - mp.TangentImpulse;
                            Vec2 Pt = appliedTangent * tangent;

                            bodyA.LinearVelocity -= invMassA * Pt;
                            bodyB.LinearVelocity += invMassB * Pt;
                            bodyA.AngularVelocity -= invIA * Vec2.Cross(rA, Pt);
                            bodyB.AngularVelocity += invIB * Vec2.Cross(rB, Pt);

                            mp = new ManifoldPoint(mp.LocalPoint, mp.NormalImpulse, newTangentImpulse, mp.Id);
                            contact.Manifold.Points[p] = mp;
                        }
                    }
                }

                // Normal constraints
                if (solvePointCount == 1)
                {
                    int index = pointCount == 2 ? primaryIndex : 0;
                    SolverPoint sp = contact.SolverPoints[index];
                    Vec2 rA = sp.RA;
                    Vec2 rB = sp.RB;
                    ManifoldPoint mp = contact.Manifold.Points[index];

                    Vec2 vA = bodyA.LinearVelocity + Vec2.Cross(bodyA.AngularVelocity, rA);
                    Vec2 vB = bodyB.LinearVelocity + Vec2.Cross(bodyB.AngularVelocity, rB);
                    Vec2 dv = vB - vA;

                    float vn = Vec2.Dot(dv, normal);
                    float impulse = -(vn - sp.VelocityBias) * sp.NormalMass;
                    float newNormalImpulse = MathF.Max(mp.NormalImpulse + impulse, 0f);
                    float appliedNormal = newNormalImpulse - mp.NormalImpulse;
                    Vec2 Pn = appliedNormal * normal;

                    bodyA.LinearVelocity -= invMassA * Pn;
                    bodyB.LinearVelocity += invMassB * Pn;
                    bodyA.AngularVelocity -= invIA * Vec2.Cross(rA, Pn);
                    bodyB.AngularVelocity += invIB * Vec2.Cross(rB, Pn);

                    mp = new ManifoldPoint(mp.LocalPoint, newNormalImpulse, mp.TangentImpulse, mp.Id);
                    contact.Manifold.Points[index] = mp;
                }
                else if (useBlockSolver)
                {
                    SolverPoint sp1 = contact.SolverPoints[0];
                    SolverPoint sp2 = contact.SolverPoints[1];
                    ManifoldPoint mp1 = contact.Manifold.Points[0];
                    ManifoldPoint mp2 = contact.Manifold.Points[1];

                    Vec2 rA1 = sp1.RA;
                    Vec2 rB1 = sp1.RB;
                    Vec2 rA2 = sp2.RA;
                    Vec2 rB2 = sp2.RB;

                    float invDet = 1f / det;
                    Mat22 normalMass = new Mat22(
                        new Vec2(invDet * k22, -invDet * k12),
                        new Vec2(-invDet * k12, invDet * k11));

                    Vec2 vA1 = bodyA.LinearVelocity + Vec2.Cross(bodyA.AngularVelocity, rA1);
                    Vec2 vB1 = bodyB.LinearVelocity + Vec2.Cross(bodyB.AngularVelocity, rB1);
                    Vec2 vA2 = bodyA.LinearVelocity + Vec2.Cross(bodyA.AngularVelocity, rA2);
                    Vec2 vB2 = bodyB.LinearVelocity + Vec2.Cross(bodyB.AngularVelocity, rB2);

                    float vn1 = Vec2.Dot(vB1 - vA1, normal);
                    float vn2 = Vec2.Dot(vB2 - vA2, normal);

                    float b1 = vn1 - sp1.VelocityBias;
                    float b2 = vn2 - sp2.VelocityBias;

                    float ax = mp1.NormalImpulse;
                    float ay = mp2.NormalImpulse;

                    // Compute b' = b - K * a
                    b1 -= k11 * ax + k12 * ay;
                    b2 -= k12 * ax + k22 * ay;

                    // Case 1: both impulses positive
                    Vec2 x = -Mat22.Mul(normalMass, new Vec2(b1, b2));
                    if (x.X >= 0f && x.Y >= 0f)
                    {
                        ApplyNormalBlock(x.X - ax, x.Y - ay, normal, rA1, rB1, rA2, rB2, ref bodyA, ref bodyB, invMassA, invIA, invMassB, invIB);
                        mp1 = new ManifoldPoint(mp1.LocalPoint, x.X, mp1.TangentImpulse, mp1.Id);
                        mp2 = new ManifoldPoint(mp2.LocalPoint, x.Y, mp2.TangentImpulse, mp2.Id);
                        contact.Manifold.Points[0] = mp1;
                        contact.Manifold.Points[1] = mp2;
                        continue;
                    }

                    // Case 2: x2 = 0
                    float x1 = k11 > 0f ? -b1 / k11 : 0f;
                    float vn2Case2 = k12 * x1 + b2;
                    if (x1 >= 0f && vn2Case2 >= 0f)
                    {
                        ApplyNormalBlock(x1 - ax, -ay, normal, rA1, rB1, rA2, rB2, ref bodyA, ref bodyB, invMassA, invIA, invMassB, invIB);
                        mp1 = new ManifoldPoint(mp1.LocalPoint, x1, mp1.TangentImpulse, mp1.Id);
                        mp2 = new ManifoldPoint(mp2.LocalPoint, 0f, mp2.TangentImpulse, mp2.Id);
                        contact.Manifold.Points[0] = mp1;
                        contact.Manifold.Points[1] = mp2;
                        continue;
                    }

                    // Case 3: x1 = 0
                    float x2 = k22 > 0f ? -b2 / k22 : 0f;
                    float vn1Case3 = k12 * x2 + b1;
                    if (x2 >= 0f && vn1Case3 >= 0f)
                    {
                        ApplyNormalBlock(-ax, x2 - ay, normal, rA1, rB1, rA2, rB2, ref bodyA, ref bodyB, invMassA, invIA, invMassB, invIB);
                        mp1 = new ManifoldPoint(mp1.LocalPoint, 0f, mp1.TangentImpulse, mp1.Id);
                        mp2 = new ManifoldPoint(mp2.LocalPoint, x2, mp2.TangentImpulse, mp2.Id);
                        contact.Manifold.Points[0] = mp1;
                        contact.Manifold.Points[1] = mp2;
                    }
                }
            }
        }

        private static void ApplyNormalBlock(
            float dx1,
            float dx2,
            Vec2 normal,
            Vec2 rA1,
            Vec2 rB1,
            Vec2 rA2,
            Vec2 rB2,
            ref Body bodyA,
            ref Body bodyB,
            float invMassA,
            float invIA,
            float invMassB,
            float invIB)
        {
            Vec2 P1 = dx1 * normal;
            Vec2 P2 = dx2 * normal;
            Vec2 P = P1 + P2;

            bodyA.LinearVelocity -= invMassA * P;
            bodyB.LinearVelocity += invMassB * P;
            bodyA.AngularVelocity -= invIA * (Vec2.Cross(rA1, P1) + Vec2.Cross(rA2, P2));
            bodyB.AngularVelocity += invIB * (Vec2.Cross(rB1, P1) + Vec2.Cross(rB2, P2));
        }

        private void SolveTOI()
        {
            if (_def.EnableContinuous == false)
            {
                return;
            }

            float timeStep = _prevTimeStep;
            int subSteps = _def.MaxSubSteps;

            for (int i = 0; i < _contacts.Count; ++i)
            {
                Contact contact = _contacts[i];
                Body? bodyA = contact.FixtureA?.Body;
                Body? bodyB = contact.FixtureB?.Body;

                if (bodyA == null || bodyB == null)
                {
                    continue;
                }

                if (!bodyA.Definition.Bullet && !bodyB.Definition.Bullet)
                {
                    continue;
                }

                ShapeProxy proxyA = ShapeGeometry.ToProxy(contact.ShapeA);
                ShapeProxy proxyB = ShapeGeometry.ToProxy(contact.ShapeB);
                ToiInput toiInput = new ToiInput(proxyA, proxyB, bodyA.Sweep, bodyB.Sweep, 1f);
                ToiOutput output = TimeOfImpact.Compute(toiInput);

                if (output.State != ToiState.Hit || output.Fraction <= 0f || output.Fraction > 1f)
                {
                    continue;
                }

                float toi = output.Fraction;
                float remaining = timeStep * (1f - toi);
                if (remaining <= 0f)
                {
                    continue;
                }

                float subDt = remaining / subSteps;
                for (int s = 0; s < subSteps; ++s)
                {
                    IntegrateForTOI(bodyA, subDt);
                    IntegrateForTOI(bodyB, subDt);

                    contact.Update(bodyA.Transform, bodyB.Transform);
                    if (contact.Manifold.PointCount == 0)
                    {
                        continue;
                    }

                    PrepareContact(contact, subDt, 1f);
                    SolveContact(contact, warmStart: true);
                    for (int it = 0; it < _def.VelocityIterations; ++it)
                    {
                        SolveContact(contact, warmStart: false);
                    }
                    for (int it = 0; it < _def.PositionIterations; ++it)
                    {
                        SolvePositionContact(contact);
                    }
                }
            }
        }

        private Body? FindBodyForShape(Shape shape)
        {
            for (int i = 0; i < _bodies.Count; ++i)
            {
                Body body = _bodies[i];
                for (int j = 0; j < body.Fixtures.Count; ++j)
                {
                    if (body.Fixtures[j].Shape == shape)
                    {
                        return body;
                    }
                }
            }
            return null;
        }

        private void IntegrateForTOI(Body body, float dt)
        {
            if (body.Type == BodyType.Static)
            {
                return;
            }

            Vec2 oldCenter = body.GetWorldCenter();
            float oldAngle = body.Transform.Q.Angle;

            if (body.Type == BodyType.Dynamic)
            {
                Vec2 accel = body.GravityScale * Gravity;
                if (body.InverseMass > 0f)
                {
                    accel += body.InverseMass * body.Force;
                }
                body.LinearVelocity += dt * accel;
                if (body.InverseInertia > 0f)
                {
                    body.AngularVelocity += dt * body.InverseInertia * body.Torque;
                }
                body.LinearVelocity = ApplyLinearDamping(body.LinearVelocity, body.LinearDamping, dt);
                body.AngularVelocity = ApplyAngularDamping(body.AngularVelocity, body.AngularDamping, dt);
            }

            Vec2 newCenter = oldCenter + dt * body.LinearVelocity;
            float newAngle = oldAngle + dt * body.AngularVelocity;
            body.SetTransformFromCenter(newCenter, newAngle);
            body.Sweep = new Sweep(body.LocalCenter, oldCenter, newCenter, oldAngle, newAngle, 0f);
        }

        private void PrepareContact(Contact contact, float timeStep, float dtRatio)
        {
            if (contact.FixtureA == null || contact.FixtureB == null)
            {
                return;
            }

            Fixture fixtureA = contact.FixtureA;
            Fixture fixtureB = contact.FixtureB;
            Body bodyA = fixtureA.Body;
            Body bodyB = fixtureB.Body;

            float radiusA = GetShapeRadius(fixtureA.Shape);
            float radiusB = GetShapeRadius(fixtureB.Shape);

            WorldManifold worldManifold = new WorldManifold();
            worldManifold.Initialize(contact.Manifold, bodyA.Transform, radiusA, bodyB.Transform, radiusB);

            Vec2 normal = worldManifold.Normal;
            Vec2 centerA = bodyA.GetWorldCenter();
            Vec2 centerB = bodyB.GetWorldCenter();
            for (int p = 0; p < contact.Manifold.PointCount; ++p)
            {
                ManifoldPoint mp = contact.Manifold.Points[p];
                if (dtRatio != 1f)
                {
                    mp = new ManifoldPoint(mp.LocalPoint, mp.NormalImpulse * dtRatio, mp.TangentImpulse * dtRatio, mp.Id);
                    contact.Manifold.Points[p] = mp;
                }

                Vec2 point = worldManifold.Points[p];
                Vec2 rA = point - centerA;
                Vec2 rB = point - centerB;

                float rnA = Vec2.Cross(rA, normal);
                float rnB = Vec2.Cross(rB, normal);
                float kNormal = bodyA.InverseMass + bodyB.InverseMass +
                                bodyA.InverseInertia * rnA * rnA + bodyB.InverseInertia * rnB * rnB;
                float normalMass = kNormal > 0f ? 1f / kNormal : 0f;

                Vec2 tangent = new Vec2(-normal.Y, normal.X);
                float rtA = Vec2.Cross(rA, tangent);
                float rtB = Vec2.Cross(rB, tangent);
                float kTangent = bodyA.InverseMass + bodyB.InverseMass +
                                 bodyA.InverseInertia * rtA * rtA + bodyB.InverseInertia * rtB * rtB;
                float tangentMass = kTangent > 0f ? 1f / kTangent : 0f;

                Vec2 vA = bodyA.LinearVelocity + Vec2.Cross(bodyA.AngularVelocity, rA);
                Vec2 vB = bodyB.LinearVelocity + Vec2.Cross(bodyB.AngularVelocity, rB);
                float vn = Vec2.Dot(vB - vA, normal);

                float velocityBias = 0f;
                float restitution = MixRestitution(fixtureA, fixtureB);
                if (vn < -_def.RestitutionThreshold)
                {
                    velocityBias = -restitution * vn;
                }
                if (_def.EnableContactSoftening)
                {
                    float separation = Vec2.Dot((centerB + rB) - (centerA + rA), normal);
                    float baumgarte = MathFng.Clamp(Constants.Baumgarte * (separation + Constants.LinearSlop) / timeStep, -Constants.MaxLinearCorrection, 0f);
                    velocityBias -= baumgarte;
                }

                contact.SolverPoints[p] = new SolverPoint
                {
                    RA = rA,
                    RB = rB,
                    NormalMass = normalMass,
                    TangentMass = tangentMass,
                    VelocityBias = velocityBias
                };
            }
        }

        private void SolveContact(Contact contact, bool warmStart)
        {
            if (contact.FixtureA == null || contact.FixtureB == null)
            {
                return;
            }

            Fixture fixtureA = contact.FixtureA;
            Fixture fixtureB = contact.FixtureB;
            Body bodyA = fixtureA.Body;
            Body bodyB = fixtureB.Body;

            if (bodyA.Type == BodyType.Static && bodyB.Type == BodyType.Static)
            {
                return;
            }

            float radiusA = GetShapeRadius(fixtureA.Shape);
            float radiusB = GetShapeRadius(fixtureB.Shape);

            WorldManifold worldManifold = new WorldManifold();
            worldManifold.Initialize(contact.Manifold, bodyA.Transform, radiusA, bodyB.Transform, radiusB);

            Vec2 normal = worldManifold.Normal;
            Vec2 tangent = new Vec2(-normal.Y, normal.X);

            float friction = MathF.Sqrt(fixtureA.Friction * fixtureB.Friction);
            float invMassA = bodyA.InverseMass;
            float invMassB = bodyB.InverseMass;
            float invIA = bodyA.InverseInertia;
            float invIB = bodyB.InverseInertia;

            for (int p = 0; p < contact.Manifold.PointCount; ++p)
            {
                SolverPoint sp = contact.SolverPoints[p];
                Vec2 rA = sp.RA;
                Vec2 rB = sp.RB;

                Vec2 vA = bodyA.LinearVelocity + Vec2.Cross(bodyA.AngularVelocity, rA);
                Vec2 vB = bodyB.LinearVelocity + Vec2.Cross(bodyB.AngularVelocity, rB);
                Vec2 dv = vB - vA;

                ManifoldPoint mp = contact.Manifold.Points[p];

                if (warmStart && (mp.NormalImpulse != 0f || mp.TangentImpulse != 0f))
                {
                    Vec2 warm = mp.NormalImpulse * normal + mp.TangentImpulse * tangent;
                    bodyA.LinearVelocity -= invMassA * warm;
                    bodyB.LinearVelocity += invMassB * warm;
                    bodyA.AngularVelocity -= invIA * Vec2.Cross(rA, warm);
                    bodyB.AngularVelocity += invIB * Vec2.Cross(rB, warm);
                }

                Vec2 vA2 = bodyA.LinearVelocity + Vec2.Cross(bodyA.AngularVelocity, rA);
                Vec2 vB2 = bodyB.LinearVelocity + Vec2.Cross(bodyB.AngularVelocity, rB);
                Vec2 dv2 = vB2 - vA2;
                float vn = Vec2.Dot(dv2, normal);
                if (vn > 0f)
                {
                    continue;
                }

                float impulse = -(vn - sp.VelocityBias) * sp.NormalMass;
                float newNormalImpulse = MathF.Max(mp.NormalImpulse + impulse, 0f);
                float appliedNormal = newNormalImpulse - mp.NormalImpulse;
                Vec2 Pn = appliedNormal * normal;
                bodyA.LinearVelocity -= invMassA * Pn;
                bodyB.LinearVelocity += invMassB * Pn;
                bodyA.AngularVelocity -= invIA * Vec2.Cross(rA, Pn);
                bodyB.AngularVelocity += invIB * Vec2.Cross(rB, Pn);

                Vec2 vA3 = bodyA.LinearVelocity + Vec2.Cross(bodyA.AngularVelocity, rA);
                Vec2 vB3 = bodyB.LinearVelocity + Vec2.Cross(bodyB.AngularVelocity, rB);
                Vec2 dv3 = vB3 - vA3;
                float vt = Vec2.Dot(dv3, tangent);

                if (sp.TangentMass > 0f)
                {
                    float tangentImpulse = -vt * sp.TangentMass;
                    float maxFriction = friction * newNormalImpulse;
                    float newTangentImpulse = MathFng.Clamp(mp.TangentImpulse + tangentImpulse, -maxFriction, maxFriction);
                    float appliedTangent = newTangentImpulse - mp.TangentImpulse;
                    Vec2 Pt = appliedTangent * tangent;

                    bodyA.LinearVelocity -= invMassA * Pt;
                    bodyB.LinearVelocity += invMassB * Pt;
                    bodyA.AngularVelocity -= invIA * Vec2.Cross(rA, Pt);
                    bodyB.AngularVelocity += invIB * Vec2.Cross(rB, Pt);

                    mp = new ManifoldPoint(mp.LocalPoint, newNormalImpulse, newTangentImpulse, mp.Id);
                }
                else
                {
                    mp = new ManifoldPoint(mp.LocalPoint, newNormalImpulse, mp.TangentImpulse, mp.Id);
                }

                contact.Manifold.Points[p] = mp;
            }
        }

        private void SolvePositionContact(Contact contact)
        {
            if (contact.FixtureA == null || contact.FixtureB == null)
            {
                return;
            }

            Fixture fixtureA = contact.FixtureA;
            Fixture fixtureB = contact.FixtureB;
            Body bodyA = fixtureA.Body;
            Body bodyB = fixtureB.Body;

            if (bodyA.Type == BodyType.Static && bodyB.Type == BodyType.Static)
            {
                return;
            }

            float radiusA = GetShapeRadius(fixtureA.Shape);
            float radiusB = GetShapeRadius(fixtureB.Shape);

            WorldManifold worldManifold = new WorldManifold();
            worldManifold.Initialize(contact.Manifold, bodyA.Transform, radiusA, bodyB.Transform, radiusB);

            Vec2 centerA = bodyA.GetWorldCenter();
            Vec2 centerB = bodyB.GetWorldCenter();
            for (int p = 0; p < contact.Manifold.PointCount; ++p)
            {
                PositionManifold pm = ComputePositionManifold(contact, bodyA.Transform, bodyB.Transform, radiusA, radiusB, p);
                Vec2 normal = pm.Normal;
                Vec2 point = pm.Point;
                float separation = pm.Separation;

                Vec2 rA = point - centerA;
                Vec2 rB = point - centerB;

                float C = MathFng.Clamp(Constants.Baumgarte * (separation + Constants.LinearSlop), -Constants.MaxLinearCorrection, 0f);

                float invMassA = bodyA.InverseMass;
                float invMassB = bodyB.InverseMass;
                float invIA = bodyA.InverseInertia;
                float invIB = bodyB.InverseInertia;

                float rnA = Vec2.Cross(rA, normal);
                float rnB = Vec2.Cross(rB, normal);
                float k = invMassA + invMassB + invIA * rnA * rnA + invIB * rnB * rnB;
                if (k <= 0f)
                {
                    continue;
                }

                float impulse = -C / k;
                Vec2 P = impulse * normal;

                Vec2 newCenterA = centerA - invMassA * P;
                Vec2 newCenterB = centerB + invMassB * P;
                float newAngleA = bodyA.Transform.Q.Angle - invIA * Vec2.Cross(rA, P);
                float newAngleB = bodyB.Transform.Q.Angle + invIB * Vec2.Cross(rB, P);

                bodyA.SetTransformFromCenter(newCenterA, newAngleA);
                bodyB.SetTransformFromCenter(newCenterB, newAngleB);
                centerA = newCenterA;
                centerB = newCenterB;
            }
        }

        private static float GetShapeRadius(Shape shape)
        {
            switch (shape.Type)
            {
                case ShapeType.Circle:
                    return ((CircleShape)shape).Radius;
                case ShapeType.Capsule:
                    return ((CapsuleShape)shape).Radius;
                case ShapeType.Segment:
                    return Constants.PolygonRadius;
                case ShapeType.Polygon:
                    return ((PolygonShape)shape).Radius;
                case ShapeType.ChainSegment:
                    return Constants.PolygonRadius;
                default:
                    return 0f;
            }
        }

        private static Vec2 ApplyLinearDamping(Vec2 velocity, float damping, float timeStep)
        {
            float factor = MathF.Max(0f, 1f - timeStep * damping);
            return velocity * factor;
        }

        private float MixFriction(Fixture a, Fixture b)
        {
            if (_def.FrictionCallback != null)
            {
                return _def.FrictionCallback(a.Friction, a.Filter.CategoryBits, b.Friction, b.Filter.CategoryBits);
            }
            return MathF.Sqrt(a.Friction * b.Friction);
        }

        private float MixRestitution(Fixture a, Fixture b)
        {
            if (_def.RestitutionCallback != null)
            {
                return _def.RestitutionCallback(a.Restitution, a.Filter.CategoryBits, b.Restitution, b.Filter.CategoryBits);
            }
            return 0.5f * (a.Restitution + b.Restitution);
        }

        private static Vec2 ClampLinearSpeed(Vec2 velocity, float maxSpeed)
        {
            float speedSqr = velocity.LengthSquared;
            float maxSqr = maxSpeed * maxSpeed;
            if (speedSqr > maxSqr && speedSqr > 0f)
            {
                float scale = maxSpeed / MathF.Sqrt(speedSqr);
                return velocity * scale;
            }

            return velocity;
        }

        private static float ClampAngularSpeed(float angularVelocity, float maxSpeed)
        {
            float abs = MathF.Abs(angularVelocity);
            if (abs > maxSpeed && abs > 0f)
            {
                return MathF.Sign(angularVelocity) * maxSpeed;
            }
            return angularVelocity;
        }

        private static Vec2 ClampTranslation(Vec2 translation, float maxTranslation)
        {
            float lengthSqr = translation.LengthSquared;
            float maxSqr = maxTranslation * maxTranslation;
            if (lengthSqr > maxSqr && lengthSqr > 0f)
            {
                float scale = maxTranslation / MathF.Sqrt(lengthSqr);
                return translation * scale;
            }
            return translation;
        }

        private static float ClampRotation(float rotation, float maxRotation)
        {
            float abs = MathF.Abs(rotation);
            if (abs > maxRotation && abs > 0f)
            {
                return MathF.Sign(rotation) * maxRotation;
            }
            return rotation;
        }

        private static float ApplyAngularDamping(float angularVelocity, float damping, float timeStep)
        {
            float factor = MathF.Max(0f, 1f - timeStep * damping);
            return angularVelocity * factor;
        }

        private static bool ShouldCollide(Fixture a, Fixture b)
        {
            if (a.IsSensor || b.IsSensor)
            {
                return false;
            }

            if (a.Filter.GroupIndex != 0 && a.Filter.GroupIndex == b.Filter.GroupIndex)
            {
                return a.Filter.GroupIndex > 0;
            }

            if ((a.Filter.MaskBits & b.Filter.CategoryBits) == 0 || (b.Filter.MaskBits & a.Filter.CategoryBits) == 0)
            {
                return false;
            }

            return true;
        }

        private void SolvePositionConstraints()
        {
            for (int i = 0; i < _contacts.Count; ++i)
            {
                Contact contact = _contacts[i];
                if (contact.FixtureA == null || contact.FixtureB == null)
                {
                    continue;
                }

                Fixture fixtureA = contact.FixtureA;
                Fixture fixtureB = contact.FixtureB;
                Body bodyA = fixtureA.Body;
                Body bodyB = fixtureB.Body;

                if (bodyA.Type == BodyType.Static && bodyB.Type == BodyType.Static)
                {
                    continue;
                }

                float radiusA = GetShapeRadius(fixtureA.Shape);
                float radiusB = GetShapeRadius(fixtureB.Shape);

                WorldManifold worldManifold = new WorldManifold();
                worldManifold.Initialize(contact.Manifold, bodyA.Transform, radiusA, bodyB.Transform, radiusB);

                Vec2 centerA = bodyA.GetWorldCenter();
                Vec2 centerB = bodyB.GetWorldCenter();
                for (int p = 0; p < contact.Manifold.PointCount; ++p)
                {
                    PositionManifold pm = ComputePositionManifold(contact, bodyA.Transform, bodyB.Transform, radiusA, radiusB, p);
                    Vec2 normal = pm.Normal;
                    Vec2 point = pm.Point;
                    float separation = pm.Separation;

                    Vec2 rA = point - centerA;
                    Vec2 rB = point - centerB;

                    float C = MathFng.Clamp(Constants.Baumgarte * (separation + Constants.LinearSlop), -Constants.MaxLinearCorrection, 0f);

                    float invMassA = bodyA.InverseMass;
                    float invMassB = bodyB.InverseMass;
                    float invIA = bodyA.InverseInertia;
                    float invIB = bodyB.InverseInertia;

                    float rnA = Vec2.Cross(rA, normal);
                    float rnB = Vec2.Cross(rB, normal);
                    float k = invMassA + invMassB + invIA * rnA * rnA + invIB * rnB * rnB;
                    if (k <= 0f)
                    {
                        continue;
                    }

                    float impulse = -C / k;
                    Vec2 P = impulse * normal;

                    Vec2 newCenterA = centerA - invMassA * P;
                    Vec2 newCenterB = centerB + invMassB * P;
                    float newAngleA = bodyA.Transform.Q.Angle - invIA * Vec2.Cross(rA, P);
                    float newAngleB = bodyB.Transform.Q.Angle + invIB * Vec2.Cross(rB, P);

                    bodyA.SetTransformFromCenter(newCenterA, newAngleA);
                    bodyB.SetTransformFromCenter(newCenterB, newAngleB);
                    centerA = newCenterA;
                    centerB = newCenterB;
                }
            }
        }

        private readonly struct PositionManifold
        {
            public readonly Vec2 Normal;
            public readonly Vec2 Point;
            public readonly float Separation;

            public PositionManifold(Vec2 normal, Vec2 point, float separation)
            {
                Normal = normal;
                Point = point;
                Separation = separation;
            }
        }

        private static PositionManifold ComputePositionManifold(Contact contact, Transform xfA, Transform xfB, float radiusA, float radiusB, int index)
        {
            Manifold manifold = contact.Manifold;
            switch (manifold.Type)
            {
                case ManifoldType.Circles:
                    {
                        Vec2 pointA = Transform.Mul(xfA, manifold.LocalPoint);
                        Vec2 pointB = Transform.Mul(xfB, manifold.Points[0].LocalPoint);
                        Vec2 normal = pointB - pointA;
                        float distance = normal.Length;
                        if (distance > Constants.Epsilon)
                        {
                            normal = normal / distance;
                        }
                        else
                        {
                            normal = new Vec2(1f, 0f);
                            distance = 0f;
                        }
                        Vec2 point = 0.5f * (pointA + pointB);
                        float separation = distance - (radiusA + radiusB);
                        return new PositionManifold(normal, point, separation);
                    }
                case ManifoldType.FaceA:
                    {
                        Vec2 normal = Rot.Mul(xfA.Q, manifold.LocalNormal);
                        Vec2 planePoint = Transform.Mul(xfA, manifold.LocalPoint);
                        Vec2 clipPoint = Transform.Mul(xfB, manifold.Points[index].LocalPoint);
                        float separation = Vec2.Dot(clipPoint - planePoint, normal) - radiusA - radiusB;
                        return new PositionManifold(normal, clipPoint, separation);
                    }
                case ManifoldType.FaceB:
                    {
                        Vec2 normal = Rot.Mul(xfB.Q, manifold.LocalNormal);
                        Vec2 planePoint = Transform.Mul(xfB, manifold.LocalPoint);
                        Vec2 clipPoint = Transform.Mul(xfA, manifold.Points[index].LocalPoint);
                        float separation = Vec2.Dot(clipPoint - planePoint, normal) - radiusA - radiusB;
                        normal = -normal;
                        return new PositionManifold(normal, clipPoint, separation);
                    }
                default:
                    return new PositionManifold(new Vec2(1f, 0f), Vec2.Zero, 0f);
            }
        }

        private void PrepareContacts(float timeStep, float dtRatio)
        {
            for (int i = 0; i < _contacts.Count; ++i)
            {
                Contact contact = _contacts[i];
                if (contact.FixtureA == null || contact.FixtureB == null)
                {
                    continue;
                }

                Fixture fixtureA = contact.FixtureA;
                Fixture fixtureB = contact.FixtureB;
                Body bodyA = fixtureA.Body;
                Body bodyB = fixtureB.Body;

                float radiusA = GetShapeRadius(fixtureA.Shape);
                float radiusB = GetShapeRadius(fixtureB.Shape);

                WorldManifold worldManifold = new WorldManifold();
                worldManifold.Initialize(contact.Manifold, bodyA.Transform, radiusA, bodyB.Transform, radiusB);

                Vec2 normal = worldManifold.Normal;
                Vec2 centerA = bodyA.GetWorldCenter();
                Vec2 centerB = bodyB.GetWorldCenter();
                for (int p = 0; p < contact.Manifold.PointCount; ++p)
                {
                    ManifoldPoint mp = contact.Manifold.Points[p];
                    if (dtRatio != 1f)
                    {
                        mp = new ManifoldPoint(mp.LocalPoint, mp.NormalImpulse * dtRatio, mp.TangentImpulse * dtRatio, mp.Id);
                        contact.Manifold.Points[p] = mp;
                    }

                    Vec2 point = worldManifold.Points[p];
                    Vec2 rA = point - centerA;
                    Vec2 rB = point - centerB;

                    float rnA = Vec2.Cross(rA, normal);
                    float rnB = Vec2.Cross(rB, normal);
                    float kNormal = bodyA.InverseMass + bodyB.InverseMass +
                                    bodyA.InverseInertia * rnA * rnA + bodyB.InverseInertia * rnB * rnB;
                    float normalMass = kNormal > 0f ? 1f / kNormal : 0f;

                    Vec2 tangent = new Vec2(-normal.Y, normal.X);
                    float rtA = Vec2.Cross(rA, tangent);
                    float rtB = Vec2.Cross(rB, tangent);
                    float kTangent = bodyA.InverseMass + bodyB.InverseMass +
                                     bodyA.InverseInertia * rtA * rtA + bodyB.InverseInertia * rtB * rtB;
                    float tangentMass = kTangent > 0f ? 1f / kTangent : 0f;

                    Vec2 vA = bodyA.LinearVelocity + Vec2.Cross(bodyA.AngularVelocity, rA);
                    Vec2 vB = bodyB.LinearVelocity + Vec2.Cross(bodyB.AngularVelocity, rB);
                    float vn = Vec2.Dot(vB - vA, normal);

                    float velocityBias = 0f;
                    float restitution = MixRestitution(fixtureA, fixtureB);
                    if (vn < -_def.RestitutionThreshold)
                    {
                        velocityBias = -restitution * vn;
                    }

                    contact.SolverPoints[p] = new SolverPoint
                    {
                        RA = rA,
                        RB = rB,
                        NormalMass = normalMass,
                        TangentMass = tangentMass,
                        VelocityBias = velocityBias
                    };
                }
            }
        }

        private void RemoveStaleContacts(System.Collections.Generic.HashSet<ContactKey> live, System.Collections.Generic.List<ContactEndEvent> endEvents)
        {
            if (_contactMap.Count == 0)
            {
                return;
            }

            System.Collections.Generic.List<ContactKey> remove = new System.Collections.Generic.List<ContactKey>();
            foreach (var pair in _contactMap)
            {
                if (!live.Contains(pair.Key))
                {
                    remove.Add(pair.Key);
                }
            }

            for (int i = 0; i < remove.Count; ++i)
            {
                if (_contactMap.TryGetValue(remove[i], out Contact? contact) && contact.FixtureA != null && contact.FixtureB != null)
                {
                    endEvents.Add(new ContactEndEvent(contact.FixtureA.UserData, contact.FixtureB.UserData));
                }
                _contactMap.Remove(remove[i]);
            }
        }

        private void RemoveStaleSensors(System.Collections.Generic.HashSet<ContactKey> live, System.Collections.Generic.List<SensorEndEvent> endEvents)
        {
            if (_sensorPairs.Count == 0)
            {
                _sensorPairs.UnionWith(live);
                return;
            }

            System.Collections.Generic.List<ContactKey> remove = new System.Collections.Generic.List<ContactKey>();
            foreach (var key in _sensorPairs)
            {
                if (!live.Contains(key))
                {
                    remove.Add(key);
                }
            }

            for (int i = 0; i < remove.Count; ++i)
            {
                _sensorPairs.Remove(remove[i]);
                if (_sensorUserData.TryGetValue(remove[i], out var data))
                {
                    endEvents.Add(new SensorEndEvent(data.A, data.B));
                    _sensorUserData.Remove(remove[i]);
                }
                else
                {
                    endEvents.Add(new SensorEndEvent(null, null));
                }
            }

            _sensorPairs.UnionWith(live);
        }

        private static bool TestOverlap(Fixture a, Fixture b)
        {
            ShapeProxy proxyA = ShapeGeometry.ToProxy(a.Shape);
            ShapeProxy proxyB = ShapeGeometry.ToProxy(b.Shape);
            DistanceInput input = new DistanceInput(proxyA, proxyB, a.Body.Transform, b.Body.Transform, true);
            DistanceOutput output = Distance.Compute(input);
            return output.Distance <= 0f;
        }

        private static bool PassesQueryFilter(Filter fixtureFilter, QueryFilter queryFilter)
        {
            if ((fixtureFilter.MaskBits & queryFilter.CategoryBits) == 0)
            {
                return false;
            }
            if ((queryFilter.MaskBits & fixtureFilter.CategoryBits) == 0)
            {
                return false;
            }
            return true;
        }

        private static bool TryRayCastFixture(Fixture fixture, RayCastInput input, out RayCastOutput output)
        {
            Transform xf = fixture.Body.Transform;
            Vec2 localOrigin = Transform.MulT(xf, input.Origin);
            Vec2 localTranslation = Rot.MulT(xf.Q, input.Translation);
            RayCastInput localInput = new RayCastInput(localOrigin, localTranslation, input.MaxFraction);

            CastOutput cast;
            switch (fixture.Shape.Type)
            {
                case ShapeType.Circle:
                    cast = Collision.RayCastCircle(ShapeGeometry.ToCircle((CircleShape)fixture.Shape), localInput);
                    break;
                case ShapeType.Capsule:
                    cast = Collision.RayCastCapsule(ShapeGeometry.ToCapsule((CapsuleShape)fixture.Shape), localInput);
                    break;
                case ShapeType.Segment:
                    cast = Collision.RayCastSegment(ShapeGeometry.ToSegment((SegmentShape)fixture.Shape), localInput);
                    break;
                case ShapeType.Polygon:
                    cast = Collision.RayCastPolygon(ShapeGeometry.ToPolygon((PolygonShape)fixture.Shape), localInput);
                    break;
                case ShapeType.ChainSegment:
                    cast = Collision.RayCastChainSegment(ShapeGeometry.ToChainSegment((ChainSegmentShape)fixture.Shape), localInput);
                    break;
                default:
                    cast = new CastOutput(Vec2.Zero, Vec2.Zero, 0f, 0, false);
                    break;
            }

            if (!cast.Hit)
            {
                output = default;
                return false;
            }

            Vec2 worldPoint = Transform.Mul(xf, cast.Point);
            Vec2 worldNormal = Rot.Mul(xf.Q, cast.Normal).Normalize();
            output = new RayCastOutput(worldPoint, worldNormal, cast.Fraction);
            return true;
        }

        public readonly struct RayCastHit
        {
            public readonly Fixture Fixture;
            public readonly Vec2 Point;
            public readonly Vec2 Normal;
            public readonly float Fraction;

            public RayCastHit(Fixture fixture, Vec2 point, Vec2 normal, float fraction)
            {
                Fixture = fixture;
                Point = point;
                Normal = normal;
                Fraction = fraction;
            }
        }

        private void AddHitEvent(Contact contact, System.Collections.Generic.List<ContactHitEvent> hitEvents)
        {
            if (contact.FixtureA == null || contact.FixtureB == null)
            {
                return;
            }

            Fixture fixtureA = contact.FixtureA;
            Fixture fixtureB = contact.FixtureB;
            Body bodyA = fixtureA.Body;
            Body bodyB = fixtureB.Body;

            float radiusA = GetShapeRadius(fixtureA.Shape);
            float radiusB = GetShapeRadius(fixtureB.Shape);
            WorldManifold worldManifold = new WorldManifold();
            worldManifold.Initialize(contact.Manifold, bodyA.Transform, radiusA, bodyB.Transform, radiusB);

            Vec2 normal = worldManifold.Normal;
            if (contact.Manifold.PointCount == 0)
            {
                return;
            }

            Vec2 point = worldManifold.Points[0];
            Vec2 centerA = bodyA.GetWorldCenter();
            Vec2 centerB = bodyB.GetWorldCenter();
            Vec2 rA = point - centerA;
            Vec2 rB = point - centerB;
            Vec2 vA = bodyA.LinearVelocity + Vec2.Cross(bodyA.AngularVelocity, rA);
            Vec2 vB = bodyB.LinearVelocity + Vec2.Cross(bodyB.AngularVelocity, rB);
            float speed = MathF.Abs(Vec2.Dot(vB - vA, normal));

            if (speed >= _def.HitEventThreshold)
            {
                hitEvents.Add(new ContactHitEvent(fixtureA.UserData, fixtureB.UserData, point, normal, speed));
            }
        }

        private void UpdateSleep(float timeStep)
        {
            float minSleepTime = float.MaxValue;

            for (int i = 0; i < _bodies.Count; ++i)
            {
                Body body = _bodies[i];
                if (body.Type == BodyType.Static || body.AllowSleep == false)
                {
                    body.SleepTime = 0f;
                    continue;
                }

                if (body.LinearVelocity.LengthSquared > Constants.LinearSleepTolerance * Constants.LinearSleepTolerance ||
                    MathF.Abs(body.AngularVelocity) > Constants.AngularSleepTolerance)
                {
                    body.SleepTime = 0f;
                    minSleepTime = 0f;
                }
                else
                {
                    body.SleepTime += timeStep;
                    minSleepTime = MathF.Min(minSleepTime, body.SleepTime);
                }
            }

            if (minSleepTime >= Constants.TimeToSleep)
            {
                for (int i = 0; i < _bodies.Count; ++i)
                {
                    Body body = _bodies[i];
                    if (body.Type == BodyType.Static || body.AllowSleep == false)
                    {
                        continue;
                    }
                    body.SetAwake(false);
                }
            }
        }

        private readonly struct ContactKey : System.IEquatable<ContactKey>
        {
            public readonly int A;
            public readonly int B;

            public ContactKey(int proxyIdA, int proxyIdB)
            {
                if (proxyIdA < proxyIdB)
                {
                    A = proxyIdA;
                    B = proxyIdB;
                }
                else
                {
                    A = proxyIdB;
                    B = proxyIdA;
                }
            }

            public bool Equals(ContactKey other) => A == other.A && B == other.B;
            public override bool Equals(object? obj) => obj is ContactKey other && Equals(other);
            public override int GetHashCode()
            {
                unchecked
                {
                    int hash = 17;
                    hash = (hash * 31) + A.GetHashCode();
                    hash = (hash * 31) + B.GetHashCode();
                    return hash;
                }
            }
        }
    }
}
