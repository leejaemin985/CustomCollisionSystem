using UnityEngine;
using Physics;

public class PhysicsObjectSweptVolumeTestCode : MonoBehaviour
{
    [SerializeField] private AttackBox attackBox;
    [SerializeField] private HitBox[] hitBoxs;

    private void Start()
    {
        attackBox.SetActive(true);

        foreach(var hitBox in hitBoxs)
        {
            hitBox.Initialize(null);
            hitBox.SetActive(true);
        }

        physicsShape1 = attackBox.currPhysicsShape.CopyClone();
        physicsShape2 = attackBox.currPhysicsShape.CopyClone();
    }

    public Transform[] targetPos;
    private int posNum = 0;

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.A))
        {
            attackBox.SetActive(false);
            attackBox.SetActive(true);
        }

        if (Input.GetKeyDown(KeyCode.T))
        {
            physicsShape1 = physicsShape2.CopyClone();

            posNum = (posNum + 1) % targetPos.Length;
            attackBox.transform.position = targetPos[posNum].position;
            attackBox.transform.rotation = targetPos[posNum].rotation;

            physicsShape2= attackBox.currPhysicsShape.CopyClone();
        }
    }

    IPhysicsShape physicsShape1 = null;
    IPhysicsShape physicsShape2 = null;

    private void OnDrawGizmos()
    {
        if (physicsShape1 == null || physicsShape2 == null) return;

        PhysicsGizmoDrawer.OnDrawGizmoPhysicsShape(physicsShape1.ComputeSweptVolume(physicsShape2), Color.green);
    }
}
