using System;
using System.Collections.Generic;
using Unity.Mathematics;

using UnityEngine;

namespace CustomPhysics
{
    [Serializable]
    public struct CollisionInfo
    {
        public bool hasCollision;
        public Vector3 contactPointA;
        public Vector3 contactPointB;

        public static CollisionInfo None => new CollisionInfo { hasCollision = false };

        public CollisionInfo(bool hasCollision, Vector3 contactPointA, Vector3 contactPointB)
        {
            this.hasCollision = hasCollision;
            this.contactPointA = contactPointA;
            this.contactPointB = contactPointB;
        }

        public CollisionInfo SwapPoint()
        {
            return new CollisionInfo(hasCollision, contactPointB, contactPointA);
        }
    }

    public static class CollisionDetecter
    {
        public static CollisionInfo CheckCollisionInfo(IPhysicsShape s1, IPhysicsShape s2)
        {
            if (s1.CollisionType == null || s2.CollisionType == null)
            {
#if UNITY_EDITOR
                Debug.LogError("[Physics] - One or both shapes have null CollisionType");
#endif
                return CollisionInfo.None;
            }

            if (TryGetCollisionHandler(s1.CollisionType, s2.CollisionType, out var handler, out bool swap))
                return swap ? handler(s2, s1).SwapPoint() : handler(s1, s2);

#if UNITY_EDITOR
            Debug.LogError($"[Physics] - Not Found CollisionCheckHandler ({s1.GetType().Name}, {s2.GetType().Name})");
#endif
            return CollisionInfo.None;
        }

        private static bool TryGetCollisionHandler(Type s1, Type s2, out CollisionCheckHandler handler, out bool swap)
        {
            if (collisionCheckHandlerMap.TryGetValue((s1, s2), out handler))
            {
                swap = false;
                return true;
            }

            if (collisionCheckHandlerMap.TryGetValue((s2, s1), out handler))
            {
                swap = true;
                return true;
            }

            swap = false;
            handler = null;
            return false;
        }



        private delegate CollisionInfo CollisionCheckHandler(IPhysicsShape shapeType1, IPhysicsShape shapeType2);

        private static readonly Dictionary<(Type, Type), CollisionCheckHandler> collisionCheckHandlerMap = new()
        {
            { (typeof(OBB), typeof(OBB)), (s1, s2) => CheckCollision_OBB_OBB((OBB)s1, (OBB)s2) },
            { (typeof(Sphere), typeof(Sphere)), (s1, s2) => CheckCollision_Sphere_Sphere((Sphere)s1, (Sphere)s2) },
            { (typeof(Capsule), typeof(Capsule)), (s1, s2) => CheckCollision_Capsule_Capsule((Capsule)s1, (Capsule)s2) },

            { (typeof(OBB), typeof(Sphere)), (s1, s2) => CheckCollision_OBB_Sphere((OBB)s1, (Sphere)s2) },
            { (typeof(OBB), typeof(Capsule)), (s1, s2) => CheckCollision_OBB_Capsule((OBB)s1, (Capsule)s2) },

            { (typeof(Sphere), typeof(OBB)), (s1, s2) => CheckCollision_Sphere_OBB((Sphere)s1, (OBB)s2) },
            { (typeof(Sphere), typeof(Capsule)), (s1, s2) => CheckCollision_Sphere_Capsule((Sphere)s1, (Capsule)s2) },

            { (typeof(Capsule), typeof(OBB)), (s1, s2) => CheckCollision_Capsule_OBB((Capsule)s1, (OBB)s2) },
            { (typeof(Capsule), typeof(Sphere)), (s1, s2) => CheckCollision_Capsule_Sphere((Capsule)s1, (Sphere)s2) },
        };

        private static CollisionInfo CheckCollision_OBB_OBB(OBB box1, OBB box2)
            => CheckOBBCollision(box1, box2);
        private static CollisionInfo CheckCollision_Sphere_Sphere(Sphere sphere1, Sphere sphere2)
            => CheckSphereCollision(sphere1, sphere2);
        private static CollisionInfo CheckCollision_Capsule_Capsule(Capsule capsule1, Capsule capsule2)
            => CheckCapsuleCollision(capsule1, capsule2);
        private static CollisionInfo CheckCollision_OBB_Sphere(OBB box, Sphere sphere)
            => CheckOBBSphereCollision(box, sphere);
        private static CollisionInfo CheckCollision_Capsule_Sphere(Capsule capsule, Sphere sphere)
            => CheckCapsuleSphereCollision(capsule, sphere);
        private static CollisionInfo CheckCollision_Sphere_OBB(Sphere sphere, OBB box)
            => CheckOBBSphereCollision(box, sphere).SwapPoint();
        private static CollisionInfo CheckCollision_Sphere_Capsule(Sphere sphere, Capsule capsule)
            => CheckCapsuleSphereCollision(capsule, sphere).SwapPoint();
        private static CollisionInfo CheckCollision_Capsule_OBB(Capsule capsule, OBB box)
            => CheckCapsuleOBBCollision(capsule, box);
        private static CollisionInfo CheckCollision_OBB_Capsule(OBB box, Capsule capsule)
            => CheckCapsuleOBBCollision(capsule, box).SwapPoint();

        #region OBB
        private static CollisionInfo CheckOBBCollision(OBB a, OBB b)
        {
            const float epsilon = 1e-6f;
            
            float3[] A = a.axis;
            float3[] B = b.axis;

            float[] aExtents = { a.halfSize.x, a.halfSize.y, a.halfSize.z };
            float[] bExtents = { b.halfSize.x, b.halfSize.y, b.halfSize.z };

            float3x3 R = float3x3.zero;
            float3x3 AbsR = float3x3.zero;

            float GetElement(float3x3 m, int row, int col)
            {
                return m[col][row];
            }

            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    R[j][i] = math.dot(A[i], B[j]);
                    AbsR[j][i] = math.abs(R[j][i]) + epsilon;
                }
            }

            float3 t = b.center - a.center;
            // 좌표를 A 기준으로 변환
            t = new float3(math.dot(t, A[0]), math.dot(t, A[1]), math.dot(t, A[2]));

            for (int i = 0; i < 3; i++)
            {
                float ra = aExtents[i];
                float rb = bExtents[0] * GetElement(AbsR, i, 0) + bExtents[1] * GetElement(AbsR, i, 1) + bExtents[2] * GetElement(AbsR, i, 2);
                if (math.abs(t[i]) > ra + rb)
                    return CollisionInfo.None;
            }

            for (int i = 0; i < 3; i++)
            {
                float ra = aExtents[0] * GetElement(AbsR, 0, i) + aExtents[1] * GetElement(AbsR, 1, i) + aExtents[2] * GetElement(AbsR, 2, i);
                float rb = bExtents[i];
                float tval = math.abs(t[0] * GetElement(R, 0, i) + t[1] * GetElement(R, 1, i) + t[2] * GetElement(R, 2, i));
                if (tval > ra + rb)
                    return CollisionInfo.None;
            }

            // 9 cross product axes (Ai x Bj)
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    float ra = aExtents[(i + 1) % 3] * GetElement(AbsR, (i + 2) % 3, j) + aExtents[(i + 2) % 3] * GetElement(AbsR, (i + 1) % 3, j);
                    float rb = bExtents[(j + 1) % 3] * GetElement(AbsR, i, (j + 2) % 3) + bExtents[(j + 2) % 3] * GetElement(AbsR, i, (j + 1) % 3);
                    float tval = math.abs(t[(i + 2) % 3] * GetElement(R, (i + 1) % 3, j) - t[(i + 1) % 3] * GetElement(R, (i + 2) % 3, j));
                    if (tval > ra + rb)
                        return CollisionInfo.None;
                }
            }
            return GetOBBCollisionInfo(a, b);
        }

        private static CollisionInfo GetOBBCollisionInfo(OBB a, OBB b)
        {
            const int maxPoints = 8;
            float3[] aPointsInB = new float3[maxPoints];
            int aCount = 0;

            float3[] bPointsInA = new float3[maxPoints];
            int bCount = 0;

            float3[] aVerts = a.GetVertices();
            float3[] bVerts = b.GetVertices();

            // A의 점들이 B 안에 있는지 확인
            for (int i = 0; i < aVerts.Length; i++)
            {
                if (IsPointInsideOBB(aVerts[i], b))
                {
                    if (aCount < maxPoints)
                        aPointsInB[aCount++] = aVerts[i];
                }
            }

            // B의 점들이 A 안에 있는지 확인
            for (int i = 0; i < bVerts.Length; i++)
            {
                if (IsPointInsideOBB(bVerts[i], a))
                {
                    if (bCount < maxPoints)
                        bPointsInA[bCount++] = bVerts[i];
                }
            }

            float3 contactA, contactB;

            if (aCount > 0 && bCount > 0)
            {
                // 양쪽 모두에 점이 있으면 평균 사용
                contactA = float3.zero;
                for (int i = 0; i < aCount; i++)
                    contactA += aPointsInB[i];
                contactA /= aCount;

                contactB = float3.zero;
                for (int i = 0; i < bCount; i++)
                    contactB += bPointsInA[i];
                contactB /= bCount;
            }
            else if (aCount > 0)
            {
                // A의 점만 B 안에 있는 경우
                contactA = float3.zero;
                for (int i = 0; i < aCount; i++)
                    contactA += aPointsInB[i];
                contactA /= aCount;

                // B의 접촉점은 A의 접촉점에서 가장 가까운 B 표면의 점
                contactB = GetClosestPointOnOBB(contactA, b);
            }
            else if (bCount > 0)
            {
                // B의 점만 A 안에 있는 경우
                contactB = float3.zero;
                for (int i = 0; i < bCount; i++)
                    contactB += bPointsInA[i];
                contactB /= bCount;

                // A의 접촉점은 B의 접촉점에서 가장 가까운 A 표면의 점
                contactA = GetClosestPointOnOBB(contactB, a);
            }
            else
            {
                // 아무 점도 내부에 없는 경우 (면이나 모서리 접촉)
                // 각 OBB에서 상대방에 가장 가까운 점 찾기
                contactA = GetClosestPointOnOBB(b.center, a);
                contactB = GetClosestPointOnOBB(a.center, b);
            }

            return new CollisionInfo(true, contactA, contactB);
        }

        private static float3 GetClosestPointOnOBB(float3 point, OBB obb)
        {
            float3 dir = point - obb.center;
            float3 closest = obb.center;

            for (int i = 0; i < 3; i++)
            {
                float projection = math.dot(dir, obb.axis[i]);
                projection = math.clamp(projection, -obb.halfSize[i], obb.halfSize[i]);
                closest += obb.axis[i] * projection;
            }

            return closest;
        }

        static bool IsPointInsideOBB(float3 point, OBB obb)
        {
            float3 dir = point - obb.center;
            for (int i = 0; i < 3; i++)
            {
                float dist = math.dot(dir, obb.axis[i]);
                if (dist > obb.halfSize[i] || dist < -obb.halfSize[i])
                    return false;
            }
            return true;
        }
        #endregion

        #region Sphere
        private static CollisionInfo CheckSphereCollision(Sphere a, Sphere b)
        {
            float3 direction = b.center - a.center;
            float distanceSq = math.lengthsq(direction);
            float radiusSum = a.radius + b.radius;

            if (distanceSq > radiusSum * radiusSum)
                return CollisionInfo.None;

            return GetSphereCollisionInfo(a, b);
        }

        private static CollisionInfo GetSphereCollisionInfo(Sphere a, Sphere b)
        {
            float3 dir = b.center - a.center;
            float dist = math.length(dir);

            if (dist < 1e-6f)
            {
                // 완전 겹쳐있는 경우, 임의 접촉점으로 센터 지정
                return new CollisionInfo(true, a.center, b.center);
            }

            float3 dirNorm = dir / dist;
            float3 contactPointA = a.center + dirNorm * a.radius;
            float3 contactPointB = b.center - dirNorm * b.radius;

            return new CollisionInfo(true, contactPointA, contactPointB);
        }
        #endregion

        #region Capsule
        private static CollisionInfo CheckCapsuleCollision(Capsule a, Capsule b)
        {
            void ClosestPointsBetweenSegments(
                float3 p1, float3 q1,
                float3 p2, float3 q2,
                out float3 closestPoint1,
                out float3 closestPoint2)
            {
                float3 d1 = q1 - p1;
                float3 d2 = q2 - p2;
                float3 r = p1 - p2;

                float a_len = math.dot(d1, d1);
                float e_len = math.dot(d2, d2);
                float f = math.dot(d2, r);

                float s, t;

                if (a_len <= math.EPSILON && e_len <= math.EPSILON)
                {
                    s = t = 0f;
                    closestPoint1 = p1;
                    closestPoint2 = p2;
                    return;
                }

                if (a_len <= math.EPSILON)
                {
                    s = 0f;
                    t = math.clamp(f / e_len, 0f, 1f);
                }
                else
                {
                    float c = math.dot(d1, r);
                    if (e_len <= math.EPSILON)
                    {
                        t = 0f;
                        s = math.clamp(-c / a_len, 0f, 1f);
                    }
                    else
                    {
                        float b = math.dot(d1, d2);
                        float denom = a_len * e_len - b * b;

                        if (denom != 0f)
                            s = math.clamp((b * f - c * e_len) / denom, 0f, 1f);
                        else
                            s = 0f;

                        t = (b * s + f) / e_len;

                        if (t < 0f)
                        {
                            t = 0f;
                            s = math.clamp(-c / a_len, 0f, 1f);
                        }
                        else if (t > 1f)
                        {
                            t = 1f;
                            s = math.clamp((b - c) / a_len, 0f, 1f);
                        }
                    }
                }

                closestPoint1 = p1 + d1 * s;
                closestPoint2 = p2 + d2 * t;
            }

            ClosestPointsBetweenSegments(a.pointA, a.pointB, b.pointA, b.pointB, out float3 pointA, out float3 pointB);

            float sqDist = math.lengthsq(pointA - pointB);
            float radiusSum = a.radius + b.radius;

            if (sqDist > radiusSum * radiusSum)
                return CollisionInfo.None;

            float3 normal;
            if (sqDist < 1e-12f)
            {
                normal = new float3(0, 1, 0);
            }
            else
            {
                normal = math.normalize(pointB - pointA);
            }

            float3 contactA = pointA + normal * a.radius;
            float3 contactB = pointB - normal * b.radius;

            return new CollisionInfo(true, contactA, contactB);
        }
        #endregion

        #region Both
        private static CollisionInfo CheckOBBSphereCollision(OBB obb, Sphere sphere)
        {
            const float epsilon = 1e-6f;

            float3 dir = sphere.center - obb.center;
            float3 closestPoint = obb.center;

            for (int i = 0; i < 3; i++)
            {
                float projection = math.dot(dir, obb.axis[i]);
                projection = math.clamp(projection, -obb.halfSize[i], obb.halfSize[i]);
                closestPoint += obb.axis[i] * projection;
            }

            float3 difference = sphere.center - closestPoint;
            float distSqr = math.lengthsq(difference);
            float radiusSq = sphere.radius * sphere.radius;

            if (distSqr > radiusSq)
                return CollisionInfo.None;

            float dist = math.sqrt(distSqr);
            float3 normal = dist > epsilon ? math.normalize(difference) : obb.axis[1];

            float3 contactPointA = closestPoint; // OBB 표면의 점
            float3 contactPointB = sphere.center - normal * sphere.radius; // 스피어 표면의 점

            return new CollisionInfo(true, contactPointA, contactPointB);
        }

        private static CollisionInfo CheckCapsuleSphereCollision(Capsule capsule, Sphere sphere)
        {
            const float epsilon = 1e-6f;

            float3 ClosestPointOnSegment(float3 a, float3 b, float3 point)
            {
                float3 ab = b - a;
                float abSqrLen = math.dot(ab, ab);
                if (abSqrLen < epsilon) return a;

                float t = math.dot(point - a, ab) / abSqrLen;
                t = math.clamp(t, 0f, 1f);
                return a + t * ab;
            }

            float3 closestPoint = ClosestPointOnSegment(capsule.pointA, capsule.pointB, sphere.center);
            float3 dir = sphere.center - closestPoint;
            float distSqr = math.lengthsq(dir);
            float radiusSum = capsule.radius + sphere.radius;
            float radiusSumSqr = radiusSum * radiusSum;

            if (distSqr > radiusSumSqr)
                return CollisionInfo.None;

            float3 normal = distSqr > epsilon ? math.normalize(dir) : new float3(0, 1, 0);

            float3 contactPointA = closestPoint + normal * capsule.radius;
            float3 contactPointB = sphere.center - normal * sphere.radius;

            return new CollisionInfo(true, contactPointA, contactPointB);
        }

        private static CollisionInfo CheckCapsuleOBBCollision(Capsule capsule, OBB obb)
        {
            const float epsilon = 1e-6f;

            // 1) 캡슐 선분을 OBB 로컬 좌표계로 변환
            float3 dA = capsule.pointA - obb.center;
            float3 dB = capsule.pointB - obb.center;

            float3x3 obbRotation = new float3x3(obb.axis[0], obb.axis[1], obb.axis[2]);
            float3 pA_local = math.mul(math.transpose(obbRotation), dA);
            float3 pB_local = math.mul(math.transpose(obbRotation), dB);

            // 2) AABB와 선분 간 최단 거리 계산
            float3 closestSeg_local, closestBox_local;
            float distSq = ClosestPtSegmentAABB(pA_local, pB_local, obb.halfSize, out closestSeg_local, out closestBox_local);

            float radiusSq = capsule.radius * capsule.radius;
            if (distSq > radiusSq)
                return CollisionInfo.None;

            float dist = math.sqrt(distSq);

            // 3) 법선 벡터 계산 (선분 → 박스 방향)
            float3 normal_local = dist > epsilon ? math.normalize(closestBox_local - closestSeg_local) : new float3(0, 1, 0);
            float3 normal_world = math.mul(obbRotation, normal_local);

            // 4) 접촉점 계산
            float3 segmentPoint_world = obb.center + math.mul(obbRotation, closestSeg_local);
            float3 boxPoint_world = obb.center + math.mul(obbRotation, closestBox_local);

            float3 contactA = boxPoint_world; // OBB의 접촉점
            float3 contactB = segmentPoint_world - normal_world * capsule.radius; // 캡슐 표면의 접촉점

            return new CollisionInfo(true, contactA, contactB);
        }

        // 보조 함수 - 샘플링 방식으로 선분-AABB 최단거리 구하기
        private static float ClosestPtSegmentAABB(float3 pA, float3 pB, float3 halfSize, out float3 closestSegPoint, out float3 closestBoxPoint)
        {
            float3 d = pB - pA;
            float segLength = math.length(d);
            float3 dir = segLength > 1e-6f ? d / segLength : float3.zero;

            float minDistSq = float.MaxValue;
            closestSegPoint = pA;
            closestBoxPoint = float3.zero;

            const int samples = 10;
            for (int i = 0; i <= samples; i++)
            {
                float t = i / (float)samples;
                float3 pt = pA + dir * segLength * t;

                float3 clamped = math.clamp(pt, -halfSize, halfSize);
                float distSq = math.lengthsq(pt - clamped);

                if (distSq < minDistSq)
                {
                    minDistSq = distSq;
                    closestSegPoint = pt;
                    closestBoxPoint = clamped;
                }
            }

            return minDistSq;
        }
        #endregion
    }
}
