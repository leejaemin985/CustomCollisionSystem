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
        public float sweepProgress; //값이 작을수록 먼저 맞은 오브젝트일 확률이 높습니다.
    }
}