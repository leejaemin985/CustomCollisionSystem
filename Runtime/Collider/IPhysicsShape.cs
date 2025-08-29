using System;
using UnityEngine;
using Unity.Mathematics;

namespace Physics
{
    public interface IPhysicsShape
    {
        float3 Center { get; }

        PhysicsObject.PhysicsShapeType ShapeType { get; }

        Type CollisionType { get; }

        IPhysicsShape ComputeSweptVolume(IPhysicsShape next);

        bool EqualsPhysicsShape(IPhysicsShape other);

        void CopyFrom(IPhysicsShape other);

        IPhysicsShape CopyClone();

        void UpdateFromTransform(Transform transform);

    }

}