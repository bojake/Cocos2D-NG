using CocosNG.Core.Input;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Numerics;
using CocosNG.Core.Primitives;
using CocosNG.Core.Actions;

namespace CocosNG.Core.Nodes
{
    internal enum CCNodeTag
    {
        Invalid = -1,
    };


    public class CCNode 
    {
        public CCPoint Position { get; set; }
        public float Z { get; set; } = 0f;
        public float Rotation { get; set; }
        public float Scale { get; set; }
        public bool Visible { get; set; } = true;

        /// <summary>
        /// Used to break z ties
        /// </summary>
        protected int __seq_id = 0;

        public CCNode? Parent { get; protected set; }

        public void RunAction(CCAction action, Func<CCNode, bool>? predicate = null)
        {
            if (predicate == null || predicate(this))
            {
                action.SetTarget(this, predicate);
                _runningActions.Add(action);
            }
        }


        public CCNode()
        {
        }


        // Touch event hooks (expressive, composable)
        public event Action<CCTouch>? OnTouchBegan;
        public event Action<CCTouch>? OnTouchMoved;
        public event Action<CCTouch>? OnTouchEnded;

        internal bool HitTest(Vector2 point)
        {
            // default: bounding box check
            return false;
        }

        internal void HandleTouchBegan(CCTouch t) => OnTouchBegan?.Invoke(t);
        internal void HandleTouchMoved(CCTouch t) => OnTouchMoved?.Invoke(t);
        internal void HandleTouchEnded(CCTouch t) => OnTouchEnded?.Invoke(t);

        public virtual CCNode Init()
        {
            return this;
        }

        /// <summary>
        /// Data assigned to this node that is controlled by the application. 
        /// </summary>
        public object UserData { get; set; } = null;

    }
}