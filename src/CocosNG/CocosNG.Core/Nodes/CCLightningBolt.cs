using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using CocosNG.Core;
using CocosNG.Core.Primitives;

namespace CocosNG.Core.Nodes
{
    public struct CCLightningBolt 
    {
        public CCPoint Start;
        public CCPoint End;
        public float Width;
        public CCColor4F BoltColor;
        public CCColor4F BoltGlowColor;
        public float StrikeTime;
        public float FadeTime;
    }
}
