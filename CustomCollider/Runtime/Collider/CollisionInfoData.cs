using System;
using System.Collections.Generic;
using UnityEngine;

namespace Physics
{
    [Serializable]
    public class CollisionInfos
    {
        public List<CollisionInfoData> collisionInfos;
        public CollisionInfos()
        {
            collisionInfos = new();
        }
    }


    [Serializable]
    public class CollisionInfoData
    {
        public HitBox hitObject;
        public Vector3 hitPoint;
        public float sweepProgress;
    }
}