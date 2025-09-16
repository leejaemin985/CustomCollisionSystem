using UnityEngine;
using Unity.Mathematics;

namespace CustomPhysics
{
    public static class SweptVolumeConfig
    {
        public static bool EnableSweptVolume = true;
        public static float MaxAllowedMovement = 3.0f;  // 이 거리 이상은 텔레포트로 간주
        public static float MinRequiredMovement = 0.05f; // 이 거리 미만은 SweptVolume 불필요
        public static bool EnableDebugLogs = false;
    }

    public static class SweptVolumeCalculator
    {
        #region Main API
        public static IPhysicsShape ComputeSweptVolume(IPhysicsShape current, IPhysicsShape next)
        {
            // SweptVolume 비활성화시 현재 위치만 반환
            if (!SweptVolumeConfig.EnableSweptVolume)
            {
                return next;
            }

            // 타입별 처리
            return (current, next) switch
            {
                (OBB a, OBB b) => ComputeSweptOBB(a, b),
                (Capsule a, Capsule b) => ComputeSweptCapsule(a, b),
                (Sphere a, Sphere b) => ComputeSweptCapsule(a, b),
                _ => next // 지원하지 않는 타입은 현재 위치 반환
            };
        }
        #endregion

        #region OBB SweptVolume
        public static OBB ComputeSweptOBB(OBB current, OBB next)
        {
            float3 movement = next.center - current.center;
            float distance = math.length(movement);

            // 움직임 검증
            if (!IsValidMovement(distance, "OBB"))
            {
                return next;
            }

            // 회전이 작으면 간단한 확장 방식 사용
            if (IsSmallRotation(current, next))
            {
                return CreateExpandedOBB(current, next);
            }

            // 회전이 크면 포인트 기반 계산
            return CreateOBBFromAllVertices(current, next);
        }

        private static bool IsSmallRotation(OBB a, OBB b)
        {
            const float rotationThreshold = 0.9f; // cos(25도) 정도

            for (int i = 0; i < 3; i++)
            {
                float dot = math.abs(math.dot(a.axis[i], b.axis[i]));
                if (dot < rotationThreshold)
                    return false;
            }
            return true;
        }

        private static OBB CreateExpandedOBB(OBB current, OBB next)
        {
            // 회전이 작으면 현재 축을 유지하고 크기만 확장
            float3 minCenter = math.min(current.center, next.center);
            float3 maxCenter = math.max(current.center, next.center);
            float3 center = (minCenter + maxCenter) * 0.5f;

            // 두 OBB를 모두 포함하는 크기 계산
            float3 halfSize = math.max(current.halfSize, next.halfSize);
            float3 centerOffset = math.abs(next.center - current.center) * 0.5f;

            // 각 축에 대해 이동량만큼 확장
            for (int i = 0; i < 3; i++)
            {
                float axisMovement = math.abs(math.dot(next.center - current.center, current.axis[i]));
                halfSize[i] += axisMovement * 0.5f;
            }

            return new OBB(center, current.axis, halfSize);
        }

        private static OBB CreateOBBFromAllVertices(OBB current, OBB next)
        {
            // 두 OBB의 모든 정점을 포함하는 최소 OBB 계산
            float3[] currentVerts = current.GetVertices();
            float3[] nextVerts = next.GetVertices();

            // 모든 점을 합친 배열
            float3[] allPoints = new float3[16];
            for (int i = 0; i < 8; i++)
            {
                allPoints[i] = currentVerts[i];
                allPoints[i + 8] = nextVerts[i];
            }

            // 간단한 AABB 기반 OBB 생성 (정확하지만 최적화되지 않을 수 있음)
            float3 min = allPoints[0];
            float3 max = allPoints[0];

            foreach (var point in allPoints)
            {
                min = math.min(min, point);
                max = math.max(max, point);
            }

            float3 center = (min + max) * 0.5f;
            float3 size = max - min;
            float3 halfSize = size * 0.5f;

            // 축은 current의 축을 유지 (더 정교한 계산도 가능하지만 일단 단순하게)
            return new OBB(center, current.axis, halfSize);
        }
        #endregion

        #region Capsule SweptVolume
        public static Capsule ComputeSweptCapsule(Capsule current, Capsule next)
        {
            float3 movement = next.center - current.center;
            float distance = math.length(movement);

            if (!IsValidMovement(distance, "Capsule"))
            {
                return next;
            }

            // 간단한 방식: 두 캡슐의 끝점을 연결하는 새로운 캡슐
            float3 startPoint = current.center;
            float3 endPoint = next.center;
            float radius = math.max(current.radius, next.radius);

            // 원래 캡슐의 길이도 고려해서 확장
            float3 currentDir = current.Direction;
            float3 nextDir = next.Direction;
            float currentHalfLength = math.distance(current.pointA, current.pointB) * 0.5f;
            float nextHalfLength = math.distance(next.pointA, next.pointB) * 0.5f;
            float maxHalfLength = math.max(currentHalfLength, nextHalfLength);

            // 시작점과 끝점을 캡슐 길이만큼 확장
            float3 finalStart = startPoint - currentDir * maxHalfLength;
            float3 finalEnd = endPoint + nextDir * maxHalfLength;

            return new Capsule(finalStart, finalEnd, radius);
        }

        public static Capsule ComputeSweptCapsule(Sphere current, Sphere next)
        {
            float3 movement = next.center - current.center;
            float distance = math.length(movement);
            float radius = current.radius;

            if (!IsValidMovement(distance, "Sphere"))
            {
                // 움직임이 없으면 작은 캡슐로 변환
                radius = math.max(current.radius, next.radius);
                float3 center = next.center;
                return new Capsule(
                    center + new float3(0, radius * 0.1f, 0),
                    center - new float3(0, radius * 0.1f, 0),
                    radius
                );
            }

            radius = math.max(current.radius, next.radius);
            return new Capsule(current.center, next.center, radius);
        }
        #endregion

        #region Utility Methods
        private static bool IsValidMovement(float distance, string shapeType)
        {
            //if (distance > SweptVolumeConfig.MaxAllowedMovement)
            //{
            //    if (SweptVolumeConfig.EnableDebugLogs)
            //    {
            //        Debug.LogWarning($"[SweptVolume] {shapeType} moved too far ({distance:F2}m), treating as teleport");
            //    }
            //    return false;
            //}

            if (distance < SweptVolumeConfig.MinRequiredMovement)
            {
                if (SweptVolumeConfig.EnableDebugLogs)
                {
                    Debug.Log($"[SweptVolume] {shapeType} moved too little ({distance:F3}m), using current position");
                }
                return false;
            }

            return true;
        }
        #endregion
    }

}