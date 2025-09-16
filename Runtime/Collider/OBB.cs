using System;
using UnityEngine;
using Unity.Mathematics;

namespace CustomPhysics
{
    public struct OBB : IPhysicsShape
    {
        public float3 Center => this.center;

        public PhysicsObject.PhysicsShapeType ShapeType => PhysicsObject.PhysicsShapeType.OBB;
        public Type CollisionType => typeof(OBB);

        public IPhysicsShape ComputeSweptVolume(IPhysicsShape next)
        {
            if (next is OBB nextOBB)
                return SweptVolumeCalculator.ComputeSweptVolume(this, nextOBB);

            return next; // 타입이 다르면 현재 위치만 사용
        }

        public float3 center;
        public float3[] axis;
        public float3 halfSize;

        private float3[] _cachedVertices;

        public OBB(Transform boxTransform)
        {
            this.center = default;
            this.axis = default;
            this.halfSize = default;
            this._cachedVertices = default;

            UpdateFromTransform(boxTransform);
        }

        public OBB(float3 center, float3 eulerRotation, float3 size)
        {
            this.center = center;

            quaternion rotation = quaternion.EulerXYZ(math.radians(eulerRotation));
            axis = new float3[3];
            axis[0] = math.mul(rotation, new float3(1, 0, 0));
            axis[1] = math.mul(rotation, new float3(0, 1, 0));
            axis[2] = math.mul(rotation, new float3(0, 0, 1));

            halfSize = size * 0.5f;

            _cachedVertices = new float3[8];
            UpdateVertices();
        }

        public OBB(float3 center, float3[] axis, float3 halfSize)
        {
            this.center = center;
            this.axis = new float3[3];
            this.axis[0] = math.normalize(axis[0]);
            this.axis[1] = math.normalize(axis[1]);
            this.axis[2] = math.normalize(axis[2]);
            this.halfSize = halfSize;

            _cachedVertices = new float3[8];
            UpdateVertices();
        }

        public void UpdateVertices()
        {
            _cachedVertices[0] = center + axis[0] * halfSize.x + axis[1] * halfSize.y + axis[2] * halfSize.z;
            _cachedVertices[1] = center + axis[0] * halfSize.x + axis[1] * halfSize.y - axis[2] * halfSize.z;
            _cachedVertices[2] = center + axis[0] * halfSize.x - axis[1] * halfSize.y + axis[2] * halfSize.z;
            _cachedVertices[3] = center + axis[0] * halfSize.x - axis[1] * halfSize.y - axis[2] * halfSize.z;
            _cachedVertices[4] = center - axis[0] * halfSize.x + axis[1] * halfSize.y + axis[2] * halfSize.z;
            _cachedVertices[5] = center - axis[0] * halfSize.x + axis[1] * halfSize.y - axis[2] * halfSize.z;
            _cachedVertices[6] = center - axis[0] * halfSize.x - axis[1] * halfSize.y + axis[2] * halfSize.z;
            _cachedVertices[7] = center - axis[0] * halfSize.x - axis[1] * halfSize.y - axis[2] * halfSize.z;
        }

        public float3[] GetVertices()
        {
            if (_cachedVertices == null) _cachedVertices = new float3[8];
            UpdateVertices();
            return _cachedVertices;
        }

        public void UpdateFromTransform(Transform transform)
        {
            center = transform.position;

            if (axis == null || axis.Length != 3)
                axis = new float3[3];

            axis[0] = math.normalize(transform.right);
            axis[1] = math.normalize(transform.up);
            axis[2] = math.normalize(transform.forward);
            halfSize = transform.lossyScale * 0.5f;

            if (_cachedVertices == null || _cachedVertices.Length != 8)
                _cachedVertices = new float3[8];

            UpdateVertices();
        }

        public bool EqualsPhysicsShape(IPhysicsShape other)
        {
            if (other is not OBB) return false;
            OBB otherOBB = (OBB)other;

            return PhysicsMath.Approximately(center, otherOBB.center) &&
                PhysicsMath.Approximately(axis, otherOBB.axis) &&
                PhysicsMath.Approximately(halfSize, otherOBB.halfSize);
        }

        public void CopyFrom(IPhysicsShape other)
        {
            if (other is not OBB) return;

            OBB otherOBB = (OBB)other;
            this.center = otherOBB.center;

            for (int index = 0, max = axis.Length; index < max; ++index)
                this.axis[index] = otherOBB.axis[index];

            this.halfSize = otherOBB.halfSize;
        }

        public IPhysicsShape CopyClone()
        {
            return new OBB(center, new float3[] { axis[0], axis[1], axis[2] }, halfSize);
        }
    }
}