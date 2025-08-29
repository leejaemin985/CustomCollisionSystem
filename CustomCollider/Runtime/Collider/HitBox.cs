using System;

namespace Physics
{
    public class HitBox : PhysicsObject
    {
        private Action<HitInfo> hitEvent;

        public void Initialize(Action<HitInfo> hitEvent)
        {
            base.Initialize();
            this.hitEvent = hitEvent;
        }

        public override PhysicsType physicsType => PhysicsType.HITABLE;

        public void OnHitEvent(HitInfo hitInfo) => hitEvent?.Invoke(hitInfo);
    }
}