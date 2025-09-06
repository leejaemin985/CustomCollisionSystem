using System;
using UnityEngine;
using Unity.Mathematics;

namespace CustomPhysics
{
    public struct Capsule : IPhysicsShape
    {
        public float3 Center => this.center;

        public PhysicsObject.PhysicsShapeType ShapeType => PhysicsObject.PhysicsShapeType.CAPSULE;
        public Type CollisionType => typeof(Capsule);

        public IPhysicsShape ComputeSweptVolume(IPhysicsShape next)
        {
            if (next is not Capsule other)
            {
                Debug.LogError("[Physics] - Shape types must match for swept volume calcuation.");
                return null;
            }

            return SweptVolumeCalculator.ComputeSweptOBBFromCapsule(this, other);
        }

        public float3 pointA;
        public float3 pointB;
        public float radius;

        public Capsule(float3 pointA, float3 pointB, float radius)
        {
            this.pointA = pointA;
            this.pointB = pointB;
            this.radius = radius;
        }

        public Capsule(Transform transform)
        {
            this.pointA = default;
            this.pointB = default;
            this.radius = default;
            UpdateFromTransform(transform);
        }

        public float3 center => (pointA + pointB) * 0.5f;

        public float Height => math.distance(pointA, pointB) + radius * 2;

        public float3 Direction => math.normalize(pointB - pointA);

        public void UpdateFromTransform(Transform transform)
        {
            float3 center = transform.position;

            float3 up = transform.up;
            float height = transform.lossyScale.y;
            float radius = math.max(transform.lossyScale.x, transform.lossyScale.z) * 0.5f;

            float halfSegment = math.max(0f, (height * 0.5f) - radius);

            this.pointA = center + up * halfSegment;
            this.pointB = center - up * halfSegment;
            this.radius = radius;
        }

        public bool EqualsPhysicsShape(IPhysicsShape other)
        {
            if (other is not Capsule) return false;

            Capsule otherCapsule = (Capsule)other;
            return PhysicsMath.Approximately(pointA, otherCapsule.pointA) &&
                PhysicsMath.Approximately(pointB, otherCapsule.pointB) &&
                PhysicsMath.Approximately(radius, otherCapsule.radius);
        }

        public void CopyFrom(IPhysicsShape other)
        {
            if (other is not Capsule) return;

            Capsule otherCapsule = (Capsule)other;
            this.pointA = otherCapsule.pointA;
            this.pointB = otherCapsule.pointB;
            this.radius = otherCapsule.radius;
        }

        public IPhysicsShape CopyClone()
        {
            return new Capsule(pointA, pointB, radius);
        }
    }
}