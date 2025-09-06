using System;

namespace CustomPhysics
{
    public class HitBox : PhysicsObject
    {
        public override PhysicsType physicsType => PhysicsType.HITABLE;

        private Action<CollisionInfoData> hitEvent;

        public void Initialize(Action<CollisionInfoData> collisionEventListener = null)
        {
            base.PhysicsInitialize();
            this.hitEvent = collisionEventListener;
        }


        public void OnHitEvent(CollisionInfoData infos) => hitEvent?.Invoke(infos);
    }
}