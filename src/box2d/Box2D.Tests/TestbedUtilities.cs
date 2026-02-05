using System;
using System.Diagnostics;
using Box2D.Collision;
using Box2D.Common;

namespace Box2D.TestBed
{
    public sealed class DebugDraw : b2Draw
    {
        public DebugDraw() : base(1)
        {
        }

        public override void DrawPolygon(b2Vec2[] vertices, int vertexCount, b2Color color)
        {
        }

        public override void DrawSolidPolygon(b2Vec2[] vertices, int vertexCount, b2Color color)
        {
        }

        public override void DrawCircle(b2Vec2 center, float radius, b2Color color)
        {
        }

        public override void DrawSolidCircle(b2Vec2 center, float radius, b2Vec2 axis, b2Color color)
        {
        }

        public override void DrawSegment(b2Vec2 p1, b2Vec2 p2, b2Color color)
        {
        }

        public override void DrawTransform(b2Transform xf)
        {
        }

        public void DrawPoint(b2Vec2 p, float size, b2Color color)
        {
        }

        public void DrawAABB(b2AABB aabb, b2Color color)
        {
        }

        public void DrawString(int x, int y, string format, params object[] args)
        {
            if (args == null || args.Length == 0)
            {
                Debug.WriteLine(format);
                return;
            }

            Debug.WriteLine(string.Format(format, args));
        }
    }

    public static class Rand
    {
        private static Random _random = new Random(12345);

        public static Random Random => _random;

        public static void Randomize(int seed)
        {
            _random = new Random(seed);
        }

        public static float RandomFloat()
        {
            return (float)_random.NextDouble();
        }

        public static float RandomFloat(float lo, float hi)
        {
            return lo + (float)_random.NextDouble() * (hi - lo);
        }
    }
}
