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



        public static OBB ComputeSweptOBBFromCapsule(Capsule a, Capsule b)
        {
            // 1) 먼저 간단한 경우들 체크
            float3 moveVec = b.center - a.center;
            float moveLen = math.length(moveVec);

            // 캡슐 축 방향들
            float3 dirA = a.Direction;  // math.normalize(a.pointB - a.pointA)
            float3 dirB = b.Direction;

            // 축 간 각도 차이 계산
            float axisDot = math.clamp(math.dot(dirA, dirB), -1f, 1f);
            float axisAngle = math.acos(math.abs(axisDot));  // 0 ~ π/2

            // 반지름 차이
            float radiusDiff = math.abs(b.radius - a.radius);
            float avgRadius = (a.radius + b.radius) * 0.5f;

            // 2) 경우별 최적화

            // Case 1: 거의 변화가 없는 경우 - 단순 확장
            if (axisAngle < math.radians(5f) &&
                moveLen < avgRadius &&
                radiusDiff < avgRadius * 0.1f)
            {
                return ComputeSimpleExpandedCapsuleOBB(a, b);
            }

            // Case 2: 평행 이동이 주된 경우
            if (axisAngle < math.radians(15f) && moveLen > avgRadius)
            {
                return ComputeTranslationDominantCapsuleOBB(a, b);
            }

            // Case 3: 회전이 주된 경우  
            if (axisAngle > math.radians(30f) && moveLen < avgRadius)
            {
                return ComputeRotationDominantCapsuleOBB(a, b);
            }

            // Case 4: 복합 케이스 - 다중 후보 테스트
            return ComputeMultiCandidateCapsuleOBB(a, b);
        }

        private static OBB ComputeSimpleExpandedCapsuleOBB(Capsule a, Capsule b)
        {
            // 거의 변화가 없으면 두 캡슐을 포함하는 최소 OBB
            float3 avgDir = math.normalize(a.Direction + b.Direction);

            // 평균 방향을 Y축으로 하는 좌표계 구성
            float3 up = avgDir;
            float3 right = math.normalize(math.cross(up, new float3(0, 0, 1)));
            if (math.lengthsq(right) < 1e-6f)
                right = math.normalize(math.cross(up, new float3(1, 0, 0)));
            float3 forward = math.normalize(math.cross(right, up));

            // 모든 관련 점들을 새 좌표계에 투영
            float3[] points = new float3[]
            {
        a.pointA, a.pointB, b.pointA, b.pointB,
        a.center, b.center
            };

            return ComputeOBBFromPointsAndAxes(points, new float3[] { right, up, forward },
                                               math.max(a.radius, b.radius));
        }

        private static OBB ComputeTranslationDominantCapsuleOBB(Capsule a, Capsule b)
        {
            // 이동이 주된 경우: 이동 방향을 고려한 축 구성
            float3 moveDir = math.normalize(b.center - a.center);
            float3 avgCapsuleDir = math.normalize(a.Direction + b.Direction);

            // 이동 방향과 캡슐 방향을 모두 고려한 최적 축 구성
            float3 primaryAxis;
            float moveDirDot = math.abs(math.dot(moveDir, avgCapsuleDir));

            if (moveDirDot > 0.8f) // 이동과 캡슐 방향이 비슷하면 캡슐 방향 우선
            {
                primaryAxis = avgCapsuleDir;
            }
            else // 다르면 이동 방향 우선
            {
                primaryAxis = moveDir;
            }

            float3 secondaryAxis = math.normalize(math.cross(primaryAxis, avgCapsuleDir));
            if (math.lengthsq(secondaryAxis) < 1e-6f)
                secondaryAxis = math.normalize(math.cross(primaryAxis, new float3(0, 1, 0)));

            float3 tertiaryAxis = math.normalize(math.cross(primaryAxis, secondaryAxis));

            float3[] points = new float3[] { a.pointA, a.pointB, b.pointA, b.pointB };

            return ComputeOBBFromPointsAndAxes(points,
                                               new float3[] { primaryAxis, secondaryAxis, tertiaryAxis },
                                               math.max(a.radius, b.radius));
        }

        private static OBB ComputeRotationDominantCapsuleOBB(Capsule a, Capsule b)
        {
            // 회전이 주된 경우: 두 캡슐의 모든 끝점을 고려

            // 회전 중심점 추정 (두 캡슐 중심의 중점)
            float3 rotCenter = (a.center + b.center) * 0.5f;

            // 회전으로 인한 확장을 고려한 포인트들
            float3[] keyPoints = new float3[]
            {
        a.pointA, a.pointB, b.pointA, b.pointB,
        // 회전 중심에서 가장 먼 점들도 추가
        a.center + math.normalize(a.center - rotCenter) * (a.Height * 0.5f + a.radius),
        b.center + math.normalize(b.center - rotCenter) * (b.Height * 0.5f + b.radius)
            };

            // 가장 적절한 축 찾기 - PCA 기반 접근
            float3 centroid = float3.zero;
            foreach (var p in keyPoints) centroid += p;
            centroid /= keyPoints.Length;

            // 공분산 행렬 기반 주축 계산 (간단 버전)
            float3 primaryDir = ComputePrimaryDirection(keyPoints, centroid);

            float3 right = math.normalize(math.cross(primaryDir, new float3(0, 1, 0)));
            if (math.lengthsq(right) < 1e-6f)
                right = math.normalize(math.cross(primaryDir, new float3(1, 0, 0)));
            float3 forward = math.normalize(math.cross(right, primaryDir));

            return ComputeOBBFromPointsAndAxes(keyPoints,
                                               new float3[] { right, primaryDir, forward },
                                               math.max(a.radius, b.radius));
        }

        private static OBB ComputeMultiCandidateCapsuleOBB(Capsule a, Capsule b)
        {
            // 여러 축 조합을 시도해서 가장 타이트한 것 선택
            OBB bestOBB = default;
            float bestVolume = float.MaxValue;

            // 후보 축들
            float3[][] axisCandidates = new float3[][]
            {
        // A 캡슐 기준
        CreateAxesFromCapsule(a),
        
        // B 캡슐 기준  
        CreateAxesFromCapsule(b),
        
        // 평균 방향 기준
        CreateAxesFromAverageDirection(a, b),
        
        // 이동 방향 기준 (이동이 충분할 때만)
        math.length(b.center - a.center) > 1e-3f ?
            CreateAxesFromMoveDirection(a, b) : null
            };

            float3[] allPoints = new float3[] { a.pointA, a.pointB, b.pointA, b.pointB };
            float maxRadius = math.max(a.radius, b.radius);

            foreach (var axes in axisCandidates)
            {
                if (axes == null) continue;

                var candidateOBB = ComputeOBBFromPointsAndAxes(allPoints, axes, maxRadius);
                float volume = candidateOBB.halfSize.x * candidateOBB.halfSize.y * candidateOBB.halfSize.z;

                if (volume < bestVolume)
                {
                    bestVolume = volume;
                    bestOBB = candidateOBB;
                }
            }

            return bestOBB;
        }

        // 헬퍼 함수들
        private static float3[] CreateAxesFromCapsule(Capsule c)
        {
            float3 up = c.Direction;
            float3 right = math.normalize(math.cross(up, new float3(0, 0, 1)));
            if (math.lengthsq(right) < 1e-6f)
                right = math.normalize(math.cross(up, new float3(1, 0, 0)));
            float3 forward = math.normalize(math.cross(right, up));

            return new float3[] { right, up, forward };
        }

        private static float3[] CreateAxesFromAverageDirection(Capsule a, Capsule b)
        {
            float3 avgDir = math.normalize(a.Direction + b.Direction);
            float3 right = math.normalize(math.cross(avgDir, new float3(0, 0, 1)));
            if (math.lengthsq(right) < 1e-6f)
                right = math.normalize(math.cross(avgDir, new float3(1, 0, 0)));
            float3 forward = math.normalize(math.cross(right, avgDir));

            return new float3[] { right, avgDir, forward };
        }

        private static float3[] CreateAxesFromMoveDirection(Capsule a, Capsule b)
        {
            float3 moveDir = math.normalize(b.center - a.center);
            float3 avgCapsuleDir = math.normalize(a.Direction + b.Direction);

            float3 right = math.normalize(math.cross(moveDir, avgCapsuleDir));
            if (math.lengthsq(right) < 1e-6f)
                right = math.normalize(math.cross(moveDir, new float3(0, 1, 0)));
            float3 forward = math.normalize(math.cross(right, moveDir));

            return new float3[] { right, moveDir, forward };
        }

        private static float3 ComputePrimaryDirection(float3[] points, float3 centroid)
        {
            // 간단한 PCA - 분산이 가장 큰 방향 찾기
            float3 bestDir = new float3(1, 0, 0);
            float maxVariance = 0f;

            // 몇 개 후보 방향으로 테스트
            float3[] candidates = new float3[]
            {
        new float3(1, 0, 0), new float3(0, 1, 0), new float3(0, 0, 1),
        math.normalize(points[0] - centroid),
        math.normalize(points[points.Length-1] - centroid)
            };

            foreach (var dir in candidates)
            {
                float variance = 0f;
                foreach (var point in points)
                {
                    float proj = math.dot(point - centroid, dir);
                    variance += proj * proj;
                }

                if (variance > maxVariance)
                {
                    maxVariance = variance;
                    bestDir = dir;
                }
            }

            return math.normalize(bestDir);
        }

        private static OBB ComputeOBBFromPointsAndAxes(float3[] points, float3[] axes, float radius)
        {
            // 축들을 정규직교화
            float3 u0 = math.normalize(axes[0]);
            float3 u1 = math.normalize(axes[1] - math.dot(axes[1], u0) * u0);
            float3 u2 = math.normalize(math.cross(u0, u1));

            // 포인트들을 축에 투영
            float3 minProj = new float3(float.PositiveInfinity);
            float3 maxProj = new float3(float.NegativeInfinity);

            foreach (var point in points)
            {
                float3 proj = new float3(
                    math.dot(point, u0),
                    math.dot(point, u1),
                    math.dot(point, u2)
                );
                minProj = math.min(minProj, proj);
                maxProj = math.max(maxProj, proj);
            }

            // 반지름만큼 확장
            float3 halfSize = 0.5f * (maxProj - minProj) + new float3(radius);
            float3 centerLocal = 0.5f * (minProj + maxProj);

            float3 center = centerLocal.x * u0 + centerLocal.y * u1 + centerLocal.z * u2;

            return new OBB(center, new float3[] { u0, u1, u2 }, halfSize);
        }



        public static Capsule ComputeSweptCapsuleFromSphere(Sphere prev, Sphere curr)
        {
            return new Capsule(prev.center, curr.center, math.max(prev.radius, curr.radius));
        }
    }

}