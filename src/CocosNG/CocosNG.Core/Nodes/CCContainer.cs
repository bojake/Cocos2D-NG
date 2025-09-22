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

    public class CCContainer : CCNode
    {

        public IList<CCNode> Children { get; }

        public void AddChild(CCNode child, float z = 0f) {
            child.Parent = this;
            child.Z = z;
            child.__seq_id = Children.Count;
            int lo = 0;
            int hi = Children.Count;

            while (lo < hi)
            {
                int mid = (lo + hi) / 2;
                if (Children[mid].Z < z)
                    lo = mid + 1;
                else
                    hi = mid;
            }

            Children.Insert(lo, child);
        }
        public void RemoveChild(CCNode child)
        {
            Children.Remove(child);
        }

        public CCContainer()
        {
            Children = new List<CCNode>();
        }

    }
}