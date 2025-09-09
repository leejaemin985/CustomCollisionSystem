using Unity.Mathematics;

namespace CustomPhysics
{
    public static class SweptVolumeCalculator
    {
        public static OBB ComputeSweptOBBFromOBB(OBB a, OBB b)
        {
            float3 moveVec = b.center - a.center;
            float moveLen = math.length(moveVec);

            // 1) 여러 축 조합을 시도해서 가장 타이트한 것 선택
            OBB bestOBB = default;
            float bestVolume = float.MaxValue;

            // 후보 축 세트들
            float3[][] axisCandidates = new float3[][]
            {
        // Case 1: A의 축 기준
        new float3[] { a.axis[0], a.axis[1], a.axis[2] },
        
        // Case 2: B의 축 기준  
        new float3[] { b.axis[0], b.axis[1], b.axis[2] },
        
        // Case 3: 이동 방향 + A축 조합 (이동이 충분할 때만)
        moveLen > 1e-3f ? new float3[] { moveVec / moveLen, a.axis[1], a.axis[2] } : null,
        
        // Case 4: 이동 방향 + B축 조합
        moveLen > 1e-3f ? new float3[] { moveVec / moveLen, b.axis[1], b.axis[2] } : null,
        
        // Case 5: A와 B축의 평균 (각도 차이가 작을 때 유효)
        new float3[] {
            math.normalize(a.axis[0] + b.axis[0]),
            math.normalize(a.axis[1] + b.axis[1]),
            math.normalize(a.axis[2] + b.axis[2])
        }
            };

            foreach (var axes in axisCandidates)
            {
                if (axes == null) continue;

                var candidateOBB = ComputeOBBWithFixedAxes(a, b, axes);
                float volume = candidateOBB.halfSize.x * candidateOBB.halfSize.y * candidateOBB.halfSize.z;

                if (volume < bestVolume)
                {
                    bestVolume = volume;
                    bestOBB = candidateOBB;
                }
            }

            return bestOBB;
        }

        private static OBB ComputeOBBWithFixedAxes(OBB a, OBB b, float3[] axes)
        {
            // 축들을 정규직교화
            float3 u0 = math.normalize(axes[0]);
            float3 u1 = math.normalize(axes[1] - math.dot(axes[1], u0) * u0);
            float3 u2 = math.normalize(math.cross(u0, u1));

            // 수치 안정성 체크
            if (math.lengthsq(u1) < 1e-6f)
            {
                u1 = math.normalize(math.cross(u0, new float3(0, 1, 0)));
                if (math.lengthsq(u1) < 1e-6f)
                    u1 = math.normalize(math.cross(u0, new float3(1, 0, 0)));
                u2 = math.normalize(math.cross(u0, u1));
            }

            // A, B 꼭짓점들을 새 축에 투영
            var va = a.GetVertices();
            var vb = b.GetVertices();

            float3 minProj = new float3(float.PositiveInfinity);
            float3 maxProj = new float3(float.NegativeInfinity);

            void ProjectPoint(float3 p)
            {
                float p0 = math.dot(p, u0);
                float p1 = math.dot(p, u1);
                float p2 = math.dot(p, u2);
                minProj = math.min(minProj, new float3(p0, p1, p2));
                maxProj = math.max(maxProj, new float3(p0, p1, p2));
            }

            for (int i = 0; i < 8; ++i)
            {
                ProjectPoint(va[i]);
                ProjectPoint(vb[i]);
            }

            float3 centerLocal = 0.5f * (minProj + maxProj);
            float3 halfSize = 0.5f * (maxProj - minProj);

            float3 center = centerLocal.x * u0 + centerLocal.y * u1 + centerLocal.z * u2;

            return new OBB(center, new float3[] { u0, u1, u2 }, halfSize);
        }

        // 추가: 각도 차이가 작을 때의 최적화된 버전
        public static OBB ComputeSweptOBBFromOBB_Optimized(OBB a, OBB b)
        {
            float3 moveVec = b.center - a.center;
            float moveLen = math.length(moveVec);

            // 각 축별 각도 차이 계산
            float maxAngleDiff = 0f;
            for (int i = 0; i < 3; i++)
            {
                float dot = math.clamp(math.dot(a.axis[i], b.axis[i]), -1f, 1f);
                float angle = math.acos(math.abs(dot));
                maxAngleDiff = math.max(maxAngleDiff, angle);
            }

            // 각도 차이가 작고 이동도 작으면 단순화된 처리
            if (maxAngleDiff < math.radians(15f) && moveLen < math.length(a.halfSize) * 0.5f)
            {
                // A, B의 평균 축을 사용하여 더 타이트하게
                float3[] avgAxes = new float3[3];
                for (int i = 0; i < 3; i++)
                {
                    // 축 방향이 반대면 하나를 뒤집어서 평균
                    float dot = math.dot(a.axis[i], b.axis[i]);
                    float3 bAxis = dot >= 0 ? b.axis[i] : -b.axis[i];
                    avgAxes[i] = math.normalize(a.axis[i] + bAxis);
                }

                return ComputeOBBWithFixedAxes(a, b, avgAxes);
            }

            // 각도 차이가 크면 기존 방식 사용
            return ComputeSweptOBBFromOBB(a, b);
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