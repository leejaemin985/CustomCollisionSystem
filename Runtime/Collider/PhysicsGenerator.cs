using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

namespace Physics
{
    public class PhysicsGenerator : MonoBehaviour
    {
        private static PhysicsGenerator instance;
        public static PhysicsGenerator Instance
        {
            get
            {
                if (instance == null)
                {
                    var obj = new GameObject("PhysicsGenerator");
                    instance = obj.AddComponent<PhysicsGenerator>();
                    DontDestroyOnLoad(obj);
                }
                return instance;
            }
        }

        private void Awake()
        {
            if (instance != null && instance != this)
            {
                Destroy(gameObject);
                return;
            }

            instance = this;
            DontDestroyOnLoad(gameObject);

            Initialized();
        }

        private void Initialized()
        {
            physicsMap = new()
            {
                { PhysicsObject.PhysicsType.ATTACK, attackPhysics = new() },
                { PhysicsObject.PhysicsType.HITABLE, hittablePhysics = new() }
            };

            collisionResult = new();
        }

        private Dictionary<PhysicsObject.PhysicsType, List<PhysicsObject>> physicsMap = default;

        private List<PhysicsObject> attackPhysics = default;
        private List<PhysicsObject> hittablePhysics = default;

        private Dictionary<AttackBox, CollisionInfos> collisionResult = default;

        public void RegisterPhysicsObject(PhysicsObject physicsObject)
        {
            if (physicsObject == null) return;

            if (physicsMap[physicsObject.physicsType].Contains(physicsObject) == true) return;
            physicsMap[physicsObject.physicsType].Add(physicsObject);
        }

        public void UnregisterPhysicsObject(PhysicsObject physicsObject)
        {
            if (physicsObject == null) return;

            if (physicsMap[physicsObject.physicsType].Contains(physicsObject) == false) return;
            physicsMap[physicsObject.physicsType].Remove(physicsObject);
        }

        private void LateUpdate()
        {
            CalculateOneTick();
        }

        private void CalculateOneTick()
        {
            collisionResult.Clear();

            foreach (AttackBox attackableOb in attackPhysics)
            {
                if (attackableOb.Active == false) continue;

                collisionResult[attackableOb] = new();

                foreach (HitBox hitableOb in hittablePhysics)
                {
                    if (hitableOb.Active == false) continue;

                    if (attackableOb.ignoreUid.Contains(hitableOb.uid) == true) continue;
                 
                    if (attackableOb.checkedHitableUIDs.Contains(hitableOb.uid) == true) continue;
                    
                    var collisionInfo = CollisionDetecter.CheckCollisionInfo(attackableOb.physicsShape, hitableOb.physicsShape);

                    if (collisionInfo.hasCollision == false) continue;

                    CollisionInfoData hitInfo = new()
                    {
                        hitObject = hitableOb,
                        hitPoint = collisionInfo.contactPointA,
                        sweepProgress = DistanceAlongMotion(attackableOb.prevPhysicsShape.Center, attackableOb.currPhysicsShape.Center, collisionInfo.contactPointA)
                    };


                    collisionResult[attackableOb].collisionInfos.Add(hitInfo);
                }
            }

            foreach (AttackBox attackableOb in collisionResult.Keys)
            {
                attackableOb.OnCollisionEvent(collisionResult[attackableOb]);
            }
        }

        private float DistanceAlongMotion(float3 prev, float3 curr, float3 contactPoint)
        {
            float3 movement = curr - prev;
            float lenSq = math.lengthsq(movement);

            if (lenSq < math.EPSILON)
            {
                return math.distance(prev, contactPoint);
            }

            float3 dir = movement / math.sqrt(lenSq);
            return math.dot(contactPoint - prev, dir);
        }
    }
}
