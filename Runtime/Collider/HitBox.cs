using System;

namespace Physics
{
    public class HitBox : PhysicsObject
    {
        public override PhysicsType physicsType => PhysicsType.HITABLE;

        private Action<CollisionInfoData> hitEvent;

        public void Initialize(Action<CollisionInfoData> collisionEventListener)
        {
            base.PhysicsInitialize();
            this.hitEvent = collisionEventListener;
        }


        public void OnHitEvent(CollisionInfoData infos) => hitEvent?.Invoke(infos);
    }
}