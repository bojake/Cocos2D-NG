using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;
using Box2D.Common;
using Box2D.Collision.Shapes;
using Box2D.Dynamics;

namespace Box2D.Viewer
{
    public sealed class WorldView : Control
    {
        private const float PixelsPerMeter = 30.0f;
        private readonly Pen _staticPen = new Pen(Color.FromArgb(140, 160, 165), 1.5f);
        private readonly Pen _dynamicPen = new Pen(Color.FromArgb(82, 170, 255), 1.5f);
        private readonly Brush _dynamicFill = new SolidBrush(Color.FromArgb(70, 82, 170, 255));
        private readonly Brush _staticFill = new SolidBrush(Color.FromArgb(50, 140, 160, 165));

        public b2World World { get; set; }

        public WorldView()
        {
            DoubleBuffered = true;
            SetStyle(ControlStyles.AllPaintingInWmPaint | ControlStyles.UserPaint | ControlStyles.OptimizedDoubleBuffer, true);
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e);

            if (World == null)
            {
                return;
            }

            Graphics g = e.Graphics;
            g.SmoothingMode = SmoothingMode.AntiAlias;

            for (b2Body body = World.BodyList; body != null; body = body.Next)
            {
                for (b2Fixture fixture = body.FixtureList; fixture != null; fixture = fixture.Next)
                {
                    DrawFixture(g, body, fixture);
                }
            }
        }

        private void DrawFixture(Graphics g, b2Body body, b2Fixture fixture)
        {
            b2Transform xf = body.Transform;
            bool isStatic = body.BodyType == b2BodyType.b2_staticBody;
            Pen pen = isStatic ? _staticPen : _dynamicPen;
            Brush fill = isStatic ? _staticFill : _dynamicFill;

            switch (fixture.ShapeType)
            {
                case b2ShapeType.e_circle:
                    DrawCircle(g, xf, (b2CircleShape)fixture.Shape, pen, fill);
                    break;
                case b2ShapeType.e_polygon:
                    DrawPolygon(g, xf, (b2PolygonShape)fixture.Shape, pen, fill);
                    break;
                case b2ShapeType.e_edge:
                    DrawEdge(g, xf, (b2EdgeShape)fixture.Shape, pen);
                    break;
                case b2ShapeType.e_chain:
                    DrawChain(g, xf, (b2ChainShape)fixture.Shape, pen);
                    break;
            }
        }

        private void DrawCircle(Graphics g, b2Transform xf, b2CircleShape circle, Pen pen, Brush fill)
        {
            b2Vec2 center = b2Math.b2Mul(ref xf, ref circle.Position);
            PointF screen = ToScreen(center);
            float radius = circle.Radius * PixelsPerMeter;
            RectangleF rect = new RectangleF(screen.X - radius, screen.Y - radius, radius * 2.0f, radius * 2.0f);
            g.FillEllipse(fill, rect);
            g.DrawEllipse(pen, rect);
        }

        private void DrawPolygon(Graphics g, b2Transform xf, b2PolygonShape polygon, Pen pen, Brush fill)
        {
            int count = polygon.VertexCount;
            if (count <= 0)
            {
                return;
            }

            PointF[] points = new PointF[count];
            for (int i = 0; i < count; ++i)
            {
                b2Vec2 vertex = polygon.Vertices[i];
                b2Vec2 worldVertex = b2Math.b2Mul(ref xf, ref vertex);
                points[i] = ToScreen(worldVertex);
            }

            g.FillPolygon(fill, points);
            g.DrawPolygon(pen, points);
        }

        private void DrawEdge(Graphics g, b2Transform xf, b2EdgeShape edge, Pen pen)
        {
            b2Vec2 v1 = b2Math.b2Mul(ref xf, ref edge.Vertex1);
            b2Vec2 v2 = b2Math.b2Mul(ref xf, ref edge.Vertex2);
            PointF p1 = ToScreen(v1);
            PointF p2 = ToScreen(v2);
            g.DrawLine(pen, p1, p2);
        }

        private void DrawChain(Graphics g, b2Transform xf, b2ChainShape chain, Pen pen)
        {
            if (chain.Vertices == null || chain.Count < 2)
            {
                return;
            }

            for (int i = 1; i < chain.Count; ++i)
            {
                b2Vec2 v1 = b2Math.b2Mul(ref xf, ref chain.Vertices[i - 1]);
                b2Vec2 v2 = b2Math.b2Mul(ref xf, ref chain.Vertices[i]);
                PointF p1 = ToScreen(v1);
                PointF p2 = ToScreen(v2);
                g.DrawLine(pen, p1, p2);
            }
        }

        private PointF ToScreen(b2Vec2 world)
        {
            float originX = ClientSize.Width * 0.5f;
            float originY = ClientSize.Height * 0.85f;
            return new PointF(originX + world.x * PixelsPerMeter, originY - world.y * PixelsPerMeter);
        }

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _staticPen.Dispose();
                _dynamicPen.Dispose();
                _dynamicFill.Dispose();
                _staticFill.Dispose();
            }
            base.Dispose(disposing);
        }
    }
}
