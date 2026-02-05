using System;
using System.Collections.Generic;
using System.Drawing;
using System.Windows.Forms;
using Box2D;
using Box2D.Dynamics;

namespace Box2D.Viewer
{
    public sealed class MainForm : Form
    {
        private readonly ComboBox _scenarioPicker;
        private readonly Button _resetButton;
        private readonly WorldView _worldView;
        private readonly Timer _timer;

        private readonly Dictionary<string, Func<b2World>> _scenarios;
        private b2World _world;

        public MainForm()
        {
            Text = "Box2D Viewer";
            ClientSize = new Size(1100, 700);
            StartPosition = FormStartPosition.CenterScreen;

            _scenarios = new Dictionary<string, Func<b2World>>
            {
                { "Stack Of Boxes", CreateStackWorld },
                { "Gravity Roller Coaster", CreateRollerCoasterWorld },
                { "Projectile (-Y Gravity)", CreateProjectileWorld }
            };

            _scenarioPicker = new ComboBox
            {
                DropDownStyle = ComboBoxStyle.DropDownList,
                Width = 260
            };
            _scenarioPicker.Items.AddRange(new object[]
            {
                "Stack Of Boxes",
                "Gravity Roller Coaster",
                "Projectile (-Y Gravity)"
            });
            _scenarioPicker.SelectedIndexChanged += (_, __) => ResetWorld();

            _resetButton = new Button
            {
                Text = "Reset",
                AutoSize = true
            };
            _resetButton.Click += (_, __) => ResetWorld();

            FlowLayoutPanel toolbar = new FlowLayoutPanel
            {
                Dock = DockStyle.Top,
                Height = 40,
                Padding = new Padding(10, 8, 10, 8)
            };
            toolbar.Controls.Add(_scenarioPicker);
            toolbar.Controls.Add(_resetButton);

            _worldView = new WorldView
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(20, 22, 25)
            };

            Controls.Add(_worldView);
            Controls.Add(toolbar);

            _scenarioPicker.SelectedIndex = 0;

            _timer = new Timer { Interval = 16 };
            _timer.Tick += (_, __) => StepWorld();
            _timer.Start();
        }

        private void ResetWorld()
        {
            string key = _scenarioPicker.SelectedItem?.ToString() ?? "Stack Of Boxes";
            if (!_scenarios.TryGetValue(key, out var factory))
            {
                factory = CreateStackWorld;
            }

            _world = factory();
            _worldView.World = _world;
            _worldView.Invalidate();
        }

        private void StepWorld()
        {
            if (_world == null)
            {
                return;
            }

            _world.Step(1.0f / 60.0f, 8, 3);
            _worldView.Invalidate();
        }

        private static b2World CreateStackWorld()
        {
            return Box2DScenarios.CreateStackOfBoxes(out _, out _);
        }

        private static b2World CreateRollerCoasterWorld()
        {
            return Box2DScenarios.CreateRollerCoaster(out _);
        }

        private static b2World CreateProjectileWorld()
        {
            return Box2DScenarios.CreateProjectile(out _);
        }
    }
}
