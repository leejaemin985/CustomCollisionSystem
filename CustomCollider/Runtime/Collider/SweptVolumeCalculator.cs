using Unity.Mathematics;

namespace Physics
{
    public static class SweptVolumeCalculator
    {
        public static OBB ComputeSweptOBBFromOBB(OBB a, OBB b)
        {
            float3 movement = b.center - a.center;
            float movementLen = math.length(movement);

            // 기준 축: 회전이 급변하지 않았다고 가정하고 a의 회전 유지
            float3[] axis = a.axis;

            // 중심점: 중간 지점
            float3 center = (a.center + b.center) * 0.5f;

            // 반지름은 두 OBB의 각 축에 대해 최대 halfExtent 선택
            float3 maxHalfSize = new float3();
            for (int i = 0; i < 3; ++i)
            {
                float extentA = a.halfSize[i];
                float extentB = b.halfSize[i];
                maxHalfSize[i] = math.max(extentA, extentB);
            }

            // movement를 각 축에 투영해서 더함 (움직인 방향에만 반응)
            for (int i = 0; i < 3; ++i)
            {
                float proj = math.abs(math.dot(movement, axis[i])) * 0.5f;
                maxHalfSize[i] += proj;
            }

            return new OBB(center, axis, maxHalfSize);
        }

        public static OBB ComputeSweptOBBFromCapsule(Capsule a, Capsule b)
        {
            // Capsule → OBB 변환 (내부 함수)
            static OBB CapsuleToOBB(Capsule capsule)
            {
                float3 dir = capsule.pointB - capsule.pointA;
                float height = math.length(dir);

                float3 up = height > 1e-5f ? dir / height : math.up();

                float3 arbitrary = math.abs(math.dot(up, math.up())) < 0.999f ? math.up() : math.right();

                float3 forward = math.normalize(math.cross(up, arbitrary));
                float3 right = math.normalize(math.cross(forward, up));
                forward = math.normalize(math.cross(right, up));

                float3[] axis = new float3[3] { right, up, forward };

                float halfHeight = height * 0.5f;

                float3 halfSize = new float3(capsule.radius, halfHeight + capsule.radius, capsule.radius);

                float3 center = (capsule.pointA + capsule.pointB) * 0.5f;

                return new OBB(center, axis, halfSize);
            }

            OBB obbA = CapsuleToOBB(a);
            OBB obbB = CapsuleToOBB(b);

            float3[] vertsA = obbA.GetVertices();
            float3[] vertsB = obbB.GetVertices();

            float3[] points = new float3[16];
            for (int i = 0; i < 8; ++i)
            {
                points[i] = vertsA[i];
                points[i + 8] = vertsB[i];
            }

            float3 movement = obbB.center - obbA.center;
            float movementLen = math.length(movement);

            if (movementLen < 1e-6f)
            {
                // 거의 움직이지 않았으면 기존 OBB 그대로 반환
                return obbA;
            }

            float3[] axis = obbA.axis;

            float3 minProj = new float3(float.PositiveInfinity);
            float3 maxProj = new float3(float.NegativeInfinity);

            foreach (var p in points)
            {
                for (int i = 0; i < 3; ++i)
                {
                    float dot = math.dot(p, axis[i]);
                    minProj[i] = math.min(minProj[i], dot);
                    maxProj[i] = math.max(maxProj[i], dot);
                }
            }

            float3 halfSize = (maxProj - minProj) * 0.5f;

            float3 center = float3.zero;
            for (int i = 0; i < 3; ++i)
            {
                center += axis[i] * ((minProj[i] + maxProj[i]) * 0.5f);
            }

            return new OBB(center, axis, halfSize);
        }

        public static Capsule ComputeSweptCapsuleFromSphere(Sphere prev, Sphere curr)
        {
            return new Capsule(prev.center, curr.center, math.max(prev.radius, curr.radius));
        }
    }

}