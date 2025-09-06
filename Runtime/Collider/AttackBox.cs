using System;
using System.Collections.Generic;

namespace CustomPhysics
{
    public class AttackBox : PhysicsObject
    {
        public override PhysicsType physicsType => PhysicsType.ATTACK;

        public HashSet<Guid> ignoreUid { get; private set; } = new();
        public HashSet<Guid> checkedHitableUIDs { get; private set; } = new();

        private Action<CollisionInfos> hitEvent;

        public void Initialize(Action<CollisionInfos> collisionEventListener = null)
        {
            base.PhysicsInitialize();
            this.hitEvent = collisionEventListener;
        }

        public void AddIgnoreUid(PhysicsObject ignorePhysics)
        {
            ignoreUid.Add(ignorePhysics.uid);
        }

        public void OnCollisionEvent(CollisionInfos hitInfos)
        {
            if (hitInfos.collisionInfos.Count == 0) return;
            this.hitEvent?.Invoke(hitInfos);

            foreach (var hitInfo in hitInfos.collisionInfos)
            {
                checkedHitableUIDs.Add(hitInfo.hitObject.uid);
            }
        }

        public override void SetActive(bool set)
        {
            base.SetActive(set);
            checkedHitableUIDs.Clear();
        }
    }
}