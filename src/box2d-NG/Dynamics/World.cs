using System;

namespace Box2DNG
{
    public sealed partial class World
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
        private readonly SolverPipeline _solverPipeline;
        private bool _islandsDirty = true;
        private readonly System.Collections.Generic.HashSet<int> _dirtyIslandIds = new System.Collections.Generic.HashSet<int>();
        private readonly System.Collections.Generic.Dictionary<Body, int> _bodyIslandIds = new System.Collections.Generic.Dictionary<Body, int>();

        public World(WorldDef def)
        {
            _def = def ?? throw new ArgumentNullException(nameof(def));
            _solverPipeline = new SolverPipeline(this);
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
        public System.Collections.Generic.IReadOnlyList<Island> LastIslands => _lastIslands;
        public SolverSet AwakeSet => _awakeSet;
        public SolverSet SleepingSet => _sleepingSet;
        public int? GetIslandId(Body body)
        {
            if (_bodyIslandIds.TryGetValue(body, out int id))
            {
                return id;
            }
            return null;
        }

        private System.Collections.Generic.List<Island> _lastIslands = new System.Collections.Generic.List<Island>();
        private readonly SolverSet _awakeSet = new SolverSet();
        private readonly SolverSet _sleepingSet = new SolverSet();

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
            MarkIslandDirty(joint.BodyA);
            MarkIslandDirty(joint.BodyB);
            HandleJointCreationAwakeState(joint.BodyA, joint.BodyB);
            return joint;
        }

        public RevoluteJoint CreateJoint(RevoluteJointDef def)
        {
            RevoluteJoint joint = new RevoluteJoint(def);
            _revoluteJoints.Add(joint);
            MarkIslandDirty(joint.BodyA);
            MarkIslandDirty(joint.BodyB);
            HandleJointCreationAwakeState(joint.BodyA, joint.BodyB);
            return joint;
        }

        public PrismaticJoint CreateJoint(PrismaticJointDef def)
        {
            PrismaticJoint joint = new PrismaticJoint(def);
            _prismaticJoints.Add(joint);
            MarkIslandDirty(joint.BodyA);
            MarkIslandDirty(joint.BodyB);
            HandleJointCreationAwakeState(joint.BodyA, joint.BodyB);
            return joint;
        }

        public WheelJoint CreateJoint(WheelJointDef def)
        {
            WheelJoint joint = new WheelJoint(def);
            _wheelJoints.Add(joint);
            MarkIslandDirty(joint.BodyA);
            MarkIslandDirty(joint.BodyB);
            HandleJointCreationAwakeState(joint.BodyA, joint.BodyB);
            return joint;
        }

        public PulleyJoint CreateJoint(PulleyJointDef def)
        {
            PulleyJoint joint = new PulleyJoint(def);
            _pulleyJoints.Add(joint);
            MarkIslandDirty(joint.BodyA);
            MarkIslandDirty(joint.BodyB);
            HandleJointCreationAwakeState(joint.BodyA, joint.BodyB);
            return joint;
        }

        public WeldJoint CreateJoint(WeldJointDef def)
        {
            WeldJoint joint = new WeldJoint(def);
            _weldJoints.Add(joint);
            MarkIslandDirty(joint.BodyA);
            MarkIslandDirty(joint.BodyB);
            HandleJointCreationAwakeState(joint.BodyA, joint.BodyB);
            return joint;
        }

        public MotorJoint CreateJoint(MotorJointDef def)
        {
            MotorJoint joint = new MotorJoint(def);
            _motorJoints.Add(joint);
            MarkIslandDirty(joint.BodyA);
            MarkIslandDirty(joint.BodyB);
            HandleJointCreationAwakeState(joint.BodyA, joint.BodyB);
            return joint;
        }

        public GearJoint CreateJoint(GearJointDef def)
        {
            GearJoint joint = new GearJoint(def);
            _gearJoints.Add(joint);
            MarkIslandDirty(joint.BodyA);
            MarkIslandDirty(joint.BodyB);
            HandleJointCreationAwakeState(joint.BodyA, joint.BodyB);
            return joint;
        }

        public RopeJoint CreateJoint(RopeJointDef def)
        {
            RopeJoint joint = new RopeJoint(def);
            _ropeJoints.Add(joint);
            MarkIslandDirty(joint.BodyA);
            MarkIslandDirty(joint.BodyB);
            HandleJointCreationAwakeState(joint.BodyA, joint.BodyB);
            return joint;
        }

        public FrictionJoint CreateJoint(FrictionJointDef def)
        {
            FrictionJoint joint = new FrictionJoint(def);
            _frictionJoints.Add(joint);
            MarkIslandDirty(joint.BodyA);
            MarkIslandDirty(joint.BodyB);
            HandleJointCreationAwakeState(joint.BodyA, joint.BodyB);
            return joint;
        }

        public bool DestroyJoint(DistanceJoint joint) => RemoveJoint(_distanceJoints, joint);
        public bool DestroyJoint(RevoluteJoint joint) => RemoveJoint(_revoluteJoints, joint);
        public bool DestroyJoint(PrismaticJoint joint) => RemoveJoint(_prismaticJoints, joint);
        public bool DestroyJoint(WheelJoint joint) => RemoveJoint(_wheelJoints, joint);
        public bool DestroyJoint(PulleyJoint joint) => RemoveJoint(_pulleyJoints, joint);
        public bool DestroyJoint(WeldJoint joint) => RemoveJoint(_weldJoints, joint);
        public bool DestroyJoint(MotorJoint joint) => RemoveJoint(_motorJoints, joint);
        public bool DestroyJoint(GearJoint joint) => RemoveJoint(_gearJoints, joint);
        public bool DestroyJoint(RopeJoint joint) => RemoveJoint(_ropeJoints, joint);
        public bool DestroyJoint(FrictionJoint joint) => RemoveJoint(_frictionJoints, joint);

        private bool RemoveJoint<TJoint>(System.Collections.Generic.List<TJoint> joints, TJoint joint) where TJoint : class
        {
            int index = joints.IndexOf(joint);
            bool removed = joints.Remove(joint);
            if (removed)
            {
                Body bodyA = GetJointBodyA(joint);
                Body bodyB = GetJointBodyB(joint);
                MarkIslandDirty(bodyA);
                MarkIslandDirty(bodyB);
                if (index >= 0)
                {
                    RemoveJointFromSolverSets(GetJointType(joint), index);
                }
                _islandsDirty = true;
            }
            return removed;
        }

        internal void NotifyAwakeChanged(Body body)
        {
            if (body.Type == BodyType.Static)
            {
                return;
            }

            MarkIslandDirty(body);

            if (body.Awake)
            {
                if (_sleepingSet.Bodies.Remove(body))
                {
                    if (!_awakeSet.Bodies.Contains(body))
                    {
                        _awakeSet.Bodies.Add(body);
                    }
                }
                else if (!_awakeSet.Bodies.Contains(body))
                {
                    _awakeSet.Bodies.Add(body);
                }

                if (_bodyIslandIds.TryGetValue(body, out int islandId) && islandId >= 0 && islandId < _lastIslands.Count)
                {
                    Island island = _lastIslands[islandId];
                    if (!island.IsAwake)
                    {
                        island.IsAwake = true;
                        MoveIslandBetweenSets(island, _sleepingSet, _awakeSet);
                        for (int i = 0; i < island.Bodies.Count; ++i)
                        {
                            Body islandBody = island.Bodies[i];
                            if (islandBody.Type == BodyType.Static)
                            {
                                continue;
                            }
                            if (!islandBody.Awake)
                            {
                                islandBody.SetAwakeFromWorld(true, notifyWorld: false);
                            }
                            if (!_awakeSet.Bodies.Contains(islandBody))
                            {
                                _awakeSet.Bodies.Add(islandBody);
                            }
                            _sleepingSet.Bodies.Remove(islandBody);
                        }
                    }
                }
            }
            else
            {
                if (_awakeSet.Bodies.Remove(body))
                {
                    if (!_sleepingSet.Bodies.Contains(body))
                    {
                        _sleepingSet.Bodies.Add(body);
                    }
                }
                else if (!_sleepingSet.Bodies.Contains(body))
                {
                    _sleepingSet.Bodies.Add(body);
                }

                if (_bodyIslandIds.TryGetValue(body, out int islandId) && islandId >= 0 && islandId < _lastIslands.Count)
                {
                    Island island = _lastIslands[islandId];
                    if (island.IsAwake)
                    {
                        island.IsAwake = false;
                        MoveIslandBetweenSets(island, _awakeSet, _sleepingSet);
                    }
                }
            }

            _islandsDirty = true;
        }

        private void MarkIslandDirty(Body body)
        {
            if (_bodyIslandIds.TryGetValue(body, out int islandId))
            {
                _dirtyIslandIds.Add(islandId);
            }
            _islandsDirty = true;
        }

        public System.Collections.Generic.IReadOnlyList<Island> BuildIslands(bool awakeOnly = true)
        {
            if (!_islandsDirty && _lastIslands.Count > 0)
            {
                return _lastIslands;
            }

            _islandsDirty = false;
            _awakeSet.Bodies.Clear();
            _awakeSet.Islands.Clear();
            _awakeSet.Contacts.Clear();
            _awakeSet.Joints.Clear();
            _sleepingSet.Bodies.Clear();
            _sleepingSet.Islands.Clear();
            _sleepingSet.Contacts.Clear();
            _sleepingSet.Joints.Clear();
            _bodyIslandIds.Clear();
            _dirtyIslandIds.Clear();

            System.Collections.Generic.Dictionary<Body, int> bodyIndex = new System.Collections.Generic.Dictionary<Body, int>(_bodies.Count);
            for (int i = 0; i < _bodies.Count; ++i)
            {
                bodyIndex[_bodies[i]] = i;
                Body body = _bodies[i];
                if (body.Type == BodyType.Static)
                {
                    continue;
                }
                if (_def.EnableSleep && body.Awake == false)
                {
                    _sleepingSet.Bodies.Add(body);
                }
                else
                {
                    _awakeSet.Bodies.Add(body);
                }
            }

            System.Collections.Generic.List<int>[] bodyContacts = new System.Collections.Generic.List<int>[_bodies.Count];
            System.Collections.Generic.List<JointHandle>[] bodyJoints = new System.Collections.Generic.List<JointHandle>[_bodies.Count];

            for (int i = 0; i < _bodies.Count; ++i)
            {
                bodyContacts[i] = new System.Collections.Generic.List<int>();
                bodyJoints[i] = new System.Collections.Generic.List<JointHandle>();
            }

            for (int i = 0; i < _contacts.Count; ++i)
            {
                Contact contact = _contacts[i];
                Fixture? fixtureA = contact.FixtureA;
                Fixture? fixtureB = contact.FixtureB;
                if (fixtureA == null || fixtureB == null)
                {
                    continue;
                }
                if (fixtureA.IsSensor || fixtureB.IsSensor)
                {
                    continue;
                }
                if (contact.Manifold.PointCount == 0)
                {
                    continue;
                }

                int ia = bodyIndex[fixtureA.Body];
                int ib = bodyIndex[fixtureB.Body];
                bodyContacts[ia].Add(i);
                bodyContacts[ib].Add(i);
            }

            AddJointHandles(bodyJoints, bodyIndex, _distanceJoints, JointType.Distance);
            AddJointHandles(bodyJoints, bodyIndex, _revoluteJoints, JointType.Revolute);
            AddJointHandles(bodyJoints, bodyIndex, _prismaticJoints, JointType.Prismatic);
            AddJointHandles(bodyJoints, bodyIndex, _wheelJoints, JointType.Wheel);
            AddJointHandles(bodyJoints, bodyIndex, _pulleyJoints, JointType.Pulley);
            AddJointHandles(bodyJoints, bodyIndex, _weldJoints, JointType.Weld);
            AddJointHandles(bodyJoints, bodyIndex, _motorJoints, JointType.Motor);
            AddJointHandles(bodyJoints, bodyIndex, _gearJoints, JointType.Gear);
            AddJointHandles(bodyJoints, bodyIndex, _ropeJoints, JointType.Rope);
            AddJointHandles(bodyJoints, bodyIndex, _frictionJoints, JointType.Friction);

            bool[] bodyVisited = new bool[_bodies.Count];
            bool[] contactAdded = new bool[_contacts.Count];
            System.Collections.Generic.Dictionary<JointHandle, bool> jointAdded = new System.Collections.Generic.Dictionary<JointHandle, bool>();

            System.Collections.Generic.List<Island> islands = new System.Collections.Generic.List<Island>();
            System.Collections.Generic.Stack<int> stack = new System.Collections.Generic.Stack<int>();
            int islandId = 0;

            for (int i = 0; i < _bodies.Count; ++i)
            {
                Body body = _bodies[i];
                if (body.Type == BodyType.Static)
                {
                    continue;
                }
                if (bodyVisited[i])
                {
                    continue;
                }

                Island island = new Island();
                stack.Push(i);
                bodyVisited[i] = true;

                while (stack.Count > 0)
                {
                    int bodyIndexValue = stack.Pop();
                    Body node = _bodies[bodyIndexValue];
                    island.Bodies.Add(node);
                    if (!_bodyIslandIds.ContainsKey(node))
                    {
                        _bodyIslandIds[node] = islandId;
                    }

                    foreach (int contactIndex in bodyContacts[bodyIndexValue])
                    {
                        if (!contactAdded[contactIndex])
                        {
                            contactAdded[contactIndex] = true;
                            island.Contacts.Add(_contacts[contactIndex]);
                        }

                        Contact contact = _contacts[contactIndex];
                        Fixture? fixtureA = contact.FixtureA;
                        Fixture? fixtureB = contact.FixtureB;
                        if (fixtureA == null || fixtureB == null)
                        {
                            continue;
                        }

                        int otherIndex = fixtureA.Body == node ? bodyIndex[fixtureB.Body] : bodyIndex[fixtureA.Body];
                        Body otherBody = _bodies[otherIndex];
                        if (otherBody.Type == BodyType.Static)
                        {
                            continue;
                        }
                        if (bodyVisited[otherIndex])
                        {
                            continue;
                        }

                        bodyVisited[otherIndex] = true;
                        stack.Push(otherIndex);
                    }

                    foreach (JointHandle handle in bodyJoints[bodyIndexValue])
                    {
                        if (!jointAdded.ContainsKey(handle))
                        {
                            jointAdded[handle] = true;
                            island.Joints.Add(handle);
                        }

                        Body? otherBody = GetJointOtherBody(handle, node);
                        if (otherBody == null)
                        {
                            continue;
                        }
                        if (otherBody.Type == BodyType.Static)
                        {
                            continue;
                        }
                        int otherIndex = bodyIndex[otherBody];
                        if (bodyVisited[otherIndex])
                        {
                            continue;
                        }

                        bodyVisited[otherIndex] = true;
                        stack.Push(otherIndex);
                    }
                }

                if (island.Bodies.Count > 0)
                {
                    bool islandAwake = !_def.EnableSleep;
                    if (_def.EnableSleep)
                    {
                        for (int j = 0; j < island.Bodies.Count; ++j)
                        {
                            if (island.Bodies[j].Awake)
                            {
                                islandAwake = true;
                                break;
                            }
                        }
                    }

                    island.IsAwake = islandAwake;
                    islands.Add(island);
                    if (islandAwake)
                    {
                        AddIslandToSet(island, _awakeSet);
                    }
                    else
                    {
                        AddIslandToSet(island, _sleepingSet);
                    }
                    islandId++;
                }
            }

            _lastIslands = islands;
            return islands;
        }

        private static void AddJointHandles<TJoint>(
            System.Collections.Generic.List<JointHandle>[] bodyJoints,
            System.Collections.Generic.Dictionary<Body, int> bodyIndex,
            System.Collections.Generic.List<TJoint> joints,
            JointType type) where TJoint : class
        {
            for (int i = 0; i < joints.Count; ++i)
            {
                TJoint joint = joints[i];
                Body bodyA = GetJointBodyA(joint);
                Body bodyB = GetJointBodyB(joint);
                int ia = bodyIndex[bodyA];
                int ib = bodyIndex[bodyB];
                JointHandle handle = new JointHandle(type, i);
                bodyJoints[ia].Add(handle);
                bodyJoints[ib].Add(handle);
            }
        }

        private static Body GetJointBodyA(object joint)
        {
            return joint switch
            {
                DistanceJoint j => j.BodyA,
                RevoluteJoint j => j.BodyA,
                PrismaticJoint j => j.BodyA,
                WheelJoint j => j.BodyA,
                PulleyJoint j => j.BodyA,
                WeldJoint j => j.BodyA,
                MotorJoint j => j.BodyA,
                GearJoint j => j.BodyA,
                RopeJoint j => j.BodyA,
                FrictionJoint j => j.BodyA,
                _ => throw new System.InvalidOperationException("Unknown joint type.")
            };
        }

        private static Body GetJointBodyB(object joint)
        {
            return joint switch
            {
                DistanceJoint j => j.BodyB,
                RevoluteJoint j => j.BodyB,
                PrismaticJoint j => j.BodyB,
                WheelJoint j => j.BodyB,
                PulleyJoint j => j.BodyB,
                WeldJoint j => j.BodyB,
                MotorJoint j => j.BodyB,
                GearJoint j => j.BodyB,
                RopeJoint j => j.BodyB,
                FrictionJoint j => j.BodyB,
                _ => throw new System.InvalidOperationException("Unknown joint type.")
            };
        }

        private void HandleJointCreationAwakeState(Body bodyA, Body bodyB)
        {
            if (!_def.EnableSleep)
            {
                return;
            }

            if (bodyA.Type == BodyType.Static && bodyB.Type == BodyType.Static)
            {
                return;
            }

            bool awakeA = bodyA.Type != BodyType.Static && bodyA.Awake;
            bool awakeB = bodyB.Type != BodyType.Static && bodyB.Awake;

            if (awakeA && !awakeB)
            {
                bodyB.SetAwake(true);
                return;
            }

            if (!awakeA && awakeB)
            {
                bodyA.SetAwake(true);
                return;
            }

            if (!awakeA && !awakeB)
            {
                MergeSleepingIslands(bodyA, bodyB);
            }
        }

        private void MergeSleepingIslands(Body bodyA, Body bodyB)
        {
            BuildIslands(awakeOnly: false);

            if (!_bodyIslandIds.TryGetValue(bodyA, out int islandAId) ||
                !_bodyIslandIds.TryGetValue(bodyB, out int islandBId) ||
                islandAId == islandBId ||
                islandAId < 0 || islandBId < 0 ||
                islandAId >= _lastIslands.Count || islandBId >= _lastIslands.Count)
            {
                return;
            }

            Island islandA = _lastIslands[islandAId];
            Island islandB = _lastIslands[islandBId];

            if (islandA.IsAwake || islandB.IsAwake)
            {
                return;
            }

            Island primary = islandA.Bodies.Count >= islandB.Bodies.Count ? islandA : islandB;
            Island secondary = primary == islandA ? islandB : islandA;

            RemoveIslandFromSet(secondary, _sleepingSet);
            _lastIslands.Remove(secondary);

            for (int i = 0; i < secondary.Bodies.Count; ++i)
            {
                Body body = secondary.Bodies[i];
                primary.Bodies.Add(body);
            }

            for (int i = 0; i < secondary.Contacts.Count; ++i)
            {
                Contact contact = secondary.Contacts[i];
                primary.Contacts.Add(contact);
            }

            for (int i = 0; i < secondary.Joints.Count; ++i)
            {
                JointHandle handle = secondary.Joints[i];
                primary.Joints.Add(handle);
            }

            AddIslandToSet(primary, _sleepingSet);
            RebuildIslandIds();
        }

        private void RebuildIslandIds()
        {
            _bodyIslandIds.Clear();
            for (int i = 0; i < _lastIslands.Count; ++i)
            {
                Island island = _lastIslands[i];
                for (int j = 0; j < island.Bodies.Count; ++j)
                {
                    _bodyIslandIds[island.Bodies[j]] = i;
                }
            }
        }

        private void AddIslandToSet(Island island, SolverSet set)
        {
            if (!set.Islands.Contains(island))
            {
                set.Islands.Add(island);
            }

            for (int i = 0; i < island.Contacts.Count; ++i)
            {
                Contact contact = island.Contacts[i];
                if (!set.Contacts.Contains(contact))
                {
                    set.Contacts.Add(contact);
                }
            }

            for (int i = 0; i < island.Joints.Count; ++i)
            {
                JointHandle handle = island.Joints[i];
                if (!set.Joints.Contains(handle))
                {
                    set.Joints.Add(handle);
                }
            }
        }

        private void RemoveIslandFromSet(Island island, SolverSet set)
        {
            set.Islands.Remove(island);

            for (int i = 0; i < island.Contacts.Count; ++i)
            {
                set.Contacts.Remove(island.Contacts[i]);
            }

            for (int i = 0; i < island.Joints.Count; ++i)
            {
                set.Joints.Remove(island.Joints[i]);
            }
        }

        private void RemoveJointFromSolverSets(JointType type, int index)
        {
            JointHandle handle = new JointHandle(type, index);
            _awakeSet.Joints.Remove(handle);
            _sleepingSet.Joints.Remove(handle);
        }

        private static JointType GetJointType(object joint)
        {
            return joint switch
            {
                DistanceJoint => JointType.Distance,
                RevoluteJoint => JointType.Revolute,
                PrismaticJoint => JointType.Prismatic,
                WheelJoint => JointType.Wheel,
                PulleyJoint => JointType.Pulley,
                WeldJoint => JointType.Weld,
                MotorJoint => JointType.Motor,
                GearJoint => JointType.Gear,
                RopeJoint => JointType.Rope,
                FrictionJoint => JointType.Friction,
                _ => throw new System.InvalidOperationException("Unknown joint type.")
            };
        }

        private void MoveIslandBetweenSets(Island island, SolverSet source, SolverSet target)
        {
            RemoveIslandFromSet(island, source);
            AddIslandToSet(island, target);
        }

        private Body? GetJointOtherBody(JointHandle handle, Body body)
        {
            return handle.Type switch
            {
                JointType.Distance => GetOtherBody(_distanceJoints[handle.Index], body),
                JointType.Revolute => GetOtherBody(_revoluteJoints[handle.Index], body),
                JointType.Prismatic => GetOtherBody(_prismaticJoints[handle.Index], body),
                JointType.Wheel => GetOtherBody(_wheelJoints[handle.Index], body),
                JointType.Pulley => GetOtherBody(_pulleyJoints[handle.Index], body),
                JointType.Weld => GetOtherBody(_weldJoints[handle.Index], body),
                JointType.Motor => GetOtherBody(_motorJoints[handle.Index], body),
                JointType.Gear => GetOtherBody(_gearJoints[handle.Index], body),
                JointType.Rope => GetOtherBody(_ropeJoints[handle.Index], body),
                JointType.Friction => GetOtherBody(_frictionJoints[handle.Index], body),
                _ => null
            };
        }

        private static Body GetOtherBody(DistanceJoint joint, Body body) => joint.BodyA == body ? joint.BodyB : joint.BodyA;
        private static Body GetOtherBody(RevoluteJoint joint, Body body) => joint.BodyA == body ? joint.BodyB : joint.BodyA;
        private static Body GetOtherBody(PrismaticJoint joint, Body body) => joint.BodyA == body ? joint.BodyB : joint.BodyA;
        private static Body GetOtherBody(WheelJoint joint, Body body) => joint.BodyA == body ? joint.BodyB : joint.BodyA;
        private static Body GetOtherBody(PulleyJoint joint, Body body) => joint.BodyA == body ? joint.BodyB : joint.BodyA;
        private static Body GetOtherBody(WeldJoint joint, Body body) => joint.BodyA == body ? joint.BodyB : joint.BodyA;
        private static Body GetOtherBody(MotorJoint joint, Body body) => joint.BodyA == body ? joint.BodyB : joint.BodyA;
        private static Body GetOtherBody(GearJoint joint, Body body) => joint.BodyA == body ? joint.BodyB : joint.BodyA;
        private static Body GetOtherBody(RopeJoint joint, Body body) => joint.BodyA == body ? joint.BodyB : joint.BodyA;
        private static Body GetOtherBody(FrictionJoint joint, Body body) => joint.BodyA == body ? joint.BodyB : joint.BodyA;

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
            bool contactsChanged = false;
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
            _awakeSet.Contacts.Clear();
            _sleepingSet.Contacts.Clear();
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
                    contactsChanged = true;
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
                        else if (_def.EnableSleep)
                        {
                            MergeSleepingIslands(bodyA, bodyB);
                        }
                    }
                }

                if (contact.IsTouching && !contact.WasTouching)
                {
                    beginEvents.Add(new ContactBeginEvent(fixtureA.UserData, fixtureB.UserData));
                    contactsChanged = true;
                    MarkIslandDirty(fixtureA.Body);
                    MarkIslandDirty(fixtureB.Body);
                }
                else if (!contact.IsTouching && contact.WasTouching)
                {
                    endEvents.Add(new ContactEndEvent(fixtureA.UserData, fixtureB.UserData));
                    contactsChanged = true;
                    MarkIslandDirty(fixtureA.Body);
                    MarkIslandDirty(fixtureB.Body);
                }

                if (contact.Manifold.PointCount > 0)
                {
                    _contacts.Add(contact);
                    if (_def.EnableSleep &&
                        fixtureA.Body.Type != BodyType.Static &&
                        fixtureB.Body.Type != BodyType.Static)
                    {
                        bool awake = fixtureA.Body.Awake || fixtureB.Body.Awake;
                        if (awake)
                        {
                            _awakeSet.Contacts.Add(contact);
                        }
                        else
                        {
                            _sleepingSet.Contacts.Add(contact);
                        }
                    }
                    else
                    {
                        _awakeSet.Contacts.Add(contact);
                    }
                    AddHitEvent(contact, hitEvents);
                }
            });

            int previousCount = _contactMap.Count;
            RemoveStaleContacts(seen, endEvents);
            if (_contactMap.Count != previousCount)
            {
                contactsChanged = true;
            }
            RemoveStaleSensors(sensorSeen, sensorEndEvents);

            if (contactsChanged)
            {
                _islandsDirty = true;
            }

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
            _solverPipeline.Step(timeStep);
        }

        private void RaiseContactImpulseEvents()
        {
            if (_awakeSet.Contacts.Count == 0)
            {
                return;
            }

            System.Collections.Generic.List<ContactImpulseEvent> impulseEvents = new System.Collections.Generic.List<ContactImpulseEvent>();
            for (int i = 0; i < _awakeSet.Contacts.Count; ++i)
            {
                Contact contact = _awakeSet.Contacts[i];
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
            SolveContacts(timeStep, warmStart, _contacts);
        }

        private void SolveContacts(float timeStep, bool warmStart, System.Collections.Generic.IReadOnlyList<Contact> contacts)
        {
            for (int i = 0; i < contacts.Count; ++i)
            {
                Contact contact = contacts[i];
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
            SolvePositionConstraints(_contacts);
        }

        private void SolvePositionConstraints(System.Collections.Generic.IReadOnlyList<Contact> contacts)
        {
            for (int i = 0; i < contacts.Count; ++i)
            {
                Contact contact = contacts[i];
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
            PrepareContacts(timeStep, dtRatio, _contacts);
        }

        private void PrepareContacts(float timeStep, float dtRatio, System.Collections.Generic.IReadOnlyList<Contact> contacts)
        {
            for (int i = 0; i < contacts.Count; ++i)
            {
                Contact contact = contacts[i];
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
                    _awakeSet.Contacts.Remove(contact);
                    _sleepingSet.Contacts.Remove(contact);
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
            if (_awakeSet.Islands.Count == 0)
            {
                return;
            }

            float linearTolSqr = Constants.LinearSleepTolerance * Constants.LinearSleepTolerance;
            float angularTol = Constants.AngularSleepTolerance;

            for (int i = 0; i < _awakeSet.Islands.Count; ++i)
            {
                Island island = _awakeSet.Islands[i];
                float minSleepTime = float.MaxValue;

                for (int j = 0; j < island.Bodies.Count; ++j)
                {
                    Body body = island.Bodies[j];
                    if (body.Type == BodyType.Static)
                    {
                        continue;
                    }

                    if (body.AllowSleep == false ||
                        body.LinearVelocity.LengthSquared > linearTolSqr ||
                        MathF.Abs(body.AngularVelocity) > angularTol)
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
                    island.IsAwake = false;
                    MoveIslandBetweenSets(island, _awakeSet, _sleepingSet);
                    i--;

                    for (int j = 0; j < island.Bodies.Count; ++j)
                    {
                        Body body = island.Bodies[j];
                        if (body.Type == BodyType.Static || body.AllowSleep == false || body.Awake == false)
                        {
                            continue;
                        }
                        body.SetAwakeFromWorld(false, notifyWorld: false);
                        _awakeSet.Bodies.Remove(body);
                        if (!_sleepingSet.Bodies.Contains(body))
                        {
                            _sleepingSet.Bodies.Add(body);
                        }
                    }
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
