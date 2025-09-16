using System;
using UnityEngine;
using Unity.Mathematics;

namespace CustomPhysics
{
    public struct Sphere : IPhysicsShape
    {
        public float3 Center => this.center;

        public PhysicsObject.PhysicsShapeType ShapeType => PhysicsObject.PhysicsShapeType.SPHERE;
        public Type CollisionType => typeof(Sphere);

        public IPhysicsShape ComputeSweptVolume(IPhysicsShape next)
        {
            return next;
            if (next is not Sphere other)
            {
                Debug.LogError("[Physics] - Shape types must match for swept volume calcuation.");
                return null;
            }

            return SweptVolumeCalculator.ComputeSweptCapsuleFromSphere(this, other);
        }

        public float3 center;
        public float radius;

        public Sphere(Transform transform)
        {
            this.center = default;
            this.radius = default;
            UpdateFromTransform(transform);
        }

        public Sphere(float3 center, float radius)
        {
            this.center = center;
            this.radius = radius;
        }

        public bool ContainsPoint(float3 point)
        {
            return math.lengthsq(point - center) <= radius * radius;
        }

        public void UpdateFromTransform(Transform transform)
        {
            center = transform.position;
            float3 scale = transform.lossyScale;
            radius = math.max(math.max(scale.x, scale.y), scale.z) * 0.5f;
        }

        public bool EqualsPhysicsShape(IPhysicsShape other)
        {
            if (other is not Sphere) return false;

            Sphere otherSphere = (Sphere)other;

            return PhysicsMath.Approximately(center, otherSphere.center) &&
                PhysicsMath.Approximately(radius, otherSphere.radius);
        }

        public void CopyFrom(IPhysicsShape other)
        {
            if (other is not Sphere) return;

            Sphere otherSphere = (Sphere)other;
            this.center = otherSphere.center;
            this.radius = otherSphere.radius;
        }

        public IPhysicsShape CopyClone()
        {
            return new Sphere(center, radius);
        }
    }
}