using Unity.Mathematics;

namespace Physics
{
    public static class SweptVolumeCalculator
    {
        public static OBB ComputeSweptOBBFromOBB(OBB a, OBB b)
        {
            float3 v = b.center - a.center;
            float vLen = math.length(v);

            // 1) 새 기준축 구성
            float3 u0, u1, u2;

            if (vLen > 1e-6f)
            {
                // 이동 방향을 첫 축으로
                u0 = v / vLen;

                // a.axis 중 u0와 가장 수직인 축을 골라 보조축 시드로
                int seed = 0;
                float minAbsDot = float.MaxValue;
                for (int i = 0; i < 3; ++i)
                {
                    float d = math.abs(math.dot(a.axis[i], u0));
                    if (d < minAbsDot) { minAbsDot = d; seed = i; }
                }

                // Gram–Schmidt로 직교화
                float3 t1 = a.axis[seed] - math.dot(a.axis[seed], u0) * u0;
                float t1Len = math.length(t1);
                if (t1Len < 1e-6f)
                {
                    // 드물게 축이 거의 평행하면 임의의 직교 벡터 선택
                    t1 = math.normalize(math.cross(u0, new float3(0, 1, 0)));
                    if (math.lengthsq(t1) < 1e-6f) t1 = math.normalize(math.cross(u0, new float3(1, 0, 0)));
                }
                u1 = math.normalize(t1);
                u2 = math.normalize(math.cross(u0, u1));
            }
            else
            {
                // 이동이 거의 없으면 a의 축을 그대로 쓰되 수치 안정화
                u0 = math.normalize(a.axis[0]);
                // a.axis[1]을 u0에 직교화
                float3 t1 = a.axis[1] - math.dot(a.axis[1], u0) * u0;
                float t1Len = math.length(t1);
                if (t1Len < 1e-6f)
                {
                    // degeneracy 방지
                    t1 = math.normalize(math.cross(u0, new float3(0, 1, 0)));
                    if (math.lengthsq(t1) < 1e-6f) t1 = math.normalize(math.cross(u0, new float3(1, 0, 0)));
                }
                u1 = math.normalize(t1);
                u2 = math.normalize(math.cross(u0, u1));
            }

            // 2) A와 B의 꼭짓점 모으기
            var va = a.GetVertices();
            var vb = b.GetVertices();

            // 3) 새 축(u0,u1,u2)에 투영해서 최소 OBB 구하기 (이 축들에 한정된 최소)
            float3 minProj = new float3(float.PositiveInfinity);
            float3 maxProj = new float3(float.NegativeInfinity);

            void AccProj(float3 p)
            {
                float p0 = math.dot(p, u0);
                float p1 = math.dot(p, u1);
                float p2 = math.dot(p, u2);
                minProj = math.min(minProj, new float3(p0, p1, p2));
                maxProj = math.max(maxProj, new float3(p0, p1, p2));
            }

            for (int i = 0; i < 8; ++i) { AccProj(va[i]); AccProj(vb[i]); }

            float3 centerLocal = 0.5f * (minProj + maxProj);
            float3 halfSize = 0.5f * (maxProj - minProj);

            // 4) 로컬 중심을 월드로 복원
            float3 center =
                centerLocal.x * u0 +
                centerLocal.y * u1 +
                centerLocal.z * u2;

            return new OBB(center, new float3[] { u0, u1, u2 }, halfSize);
        }

        public static OBB ComputeSweptOBBFromCapsule(Capsule a, Capsule b)
        {
            // --- Capsule → OBB: 반지름과 축을 보존하는 타이트한 근사 ---
            static OBB CapsuleToOBB(Capsule c)
            {
                float3 upDir = c.pointB - c.pointA;
                float h = math.length(upDir);

                float3 up = h > 1e-6f ? upDir / h : new float3(0, 1, 0);

                // 임의 직교 벡터 선택
                float3 pick = math.abs(math.dot(up, new float3(0, 1, 0))) < 0.999f
                    ? new float3(0, 1, 0)
                    : new float3(1, 0, 0);

                float3 forward = math.normalize(math.cross(up, pick));
                float3 right = math.normalize(math.cross(forward, up));
                forward = math.normalize(math.cross(right, up)); // 수치 안정화

                // 캡슐 외접 OBB halfSize: X/Z는 radius, Y는 (halfHeight + radius)
                float halfHeight = h * 0.5f;
                float3 halfSize = new float3(c.radius, halfHeight + c.radius, c.radius);

                float3 center = (c.pointA + c.pointB) * 0.5f;

                return new OBB(center, new float3[] { right, up, forward }, halfSize);
            }

            OBB obbA = CapsuleToOBB(a);
            OBB obbB = CapsuleToOBB(b);

            // A/B 꼭짓점 수집
            var va = obbA.GetVertices();
            var vb = obbB.GetVertices();

            float3[] pts = new float3[16];
            for (int i = 0; i < 8; ++i) { pts[i] = va[i]; pts[i + 8] = vb[i]; }

            // --- 새 기준축 구성: 이동 방향 우선 ---
            float3 move = obbB.center - obbA.center;
            float mLen = math.length(move);

            float3 u0, u1, u2;
            if (mLen > 1e-6f)
            {
                u0 = move / mLen; // 이동 방향

                // obbA.axis 중 u0와 가장 수직인 축을 시드로 골라 직교화
                int seed = 0;
                float minAbsDot = float.MaxValue;
                for (int i = 0; i < 3; ++i)
                {
                    float d = math.abs(math.dot(obbA.axis[i], u0));
                    if (d < minAbsDot) { minAbsDot = d; seed = i; }
                }

                float3 t1 = obbA.axis[seed] - math.dot(obbA.axis[seed], u0) * u0;
                float l1 = math.length(t1);
                if (l1 < 1e-6f)
                {
                    // 드문 병렬 특이상황 회피
                    t1 = math.normalize(math.cross(u0, new float3(0, 1, 0)));
                    if (math.lengthsq(t1) < 1e-6f) t1 = math.normalize(math.cross(u0, new float3(1, 0, 0)));
                }
                u1 = math.normalize(t1);
                u2 = math.normalize(math.cross(u0, u1));
            }
            else
            {
                // 이동 거의 없음: obbA 기준으로 안정적인 직교기저 생성
                u0 = math.normalize(obbA.axis[0]);

                float3 t1 = obbA.axis[1] - math.dot(obbA.axis[1], u0) * u0;
                float l1 = math.length(t1);
                if (l1 < 1e-6f)
                {
                    t1 = math.normalize(math.cross(u0, new float3(0, 1, 0)));
                    if (math.lengthsq(t1) < 1e-6f) t1 = math.normalize(math.cross(u0, new float3(1, 0, 0)));
                }
                u1 = math.normalize(t1);
                u2 = math.normalize(math.cross(u0, u1));
            }

            // --- 투영으로 최소 바운딩(해당 축계 한정) ---
            float3 minP = new float3(float.PositiveInfinity);
            float3 maxP = new float3(float.NegativeInfinity);

            void Acc(float3 p)
            {
                float p0 = math.dot(p, u0);
                float p1 = math.dot(p, u1);
                float p2 = math.dot(p, u2);
                minP = math.min(minP, new float3(p0, p1, p2));
                maxP = math.max(maxP, new float3(p0, p1, p2));
            }

            for (int i = 0; i < 16; ++i) Acc(pts[i]);

            float3 centerLocal = 0.5f * (minP + maxP);
            float3 halfSize = 0.5f * (maxP - minP);

            float3 center =
                centerLocal.x * u0 +
                centerLocal.y * u1 +
                centerLocal.z * u2;

            return new OBB(center, new float3[] { u0, u1, u2 }, halfSize);
        }


        public static Capsule ComputeSweptCapsuleFromSphere(Sphere prev, Sphere curr)
        {
            return new Capsule(prev.center, curr.center, math.max(prev.radius, curr.radius));
        }
    }

}