using System;
using UnityEngine;
using UnityEditor;
using Unity.Mathematics;

namespace Physics
{
    public abstract class PhysicsObject : MonoBehaviour
    {
        public Guid uid { get; private set; } = Guid.NewGuid();

        public override bool Equals(object obj)
        {
            return obj is PhysicsObject physicObj && physicObj.uid == this.uid; 
        }

        public override int GetHashCode() => uid.GetHashCode();

        protected bool active = false;

        public virtual bool Active => active && gameObject.activeSelf;

        public virtual void SetActive(bool set)
        {
            active = set;
        }

        public enum PhysicsShapeType
        {
            OBB,
            SPHERE,
            CAPSULE
        }

        public enum PhysicsType
        {
            ATTACK,
            HITABLE
        }

        public abstract PhysicsType physicsType { get; }

        public PhysicsShapeType physicsShapeType;
        
        public IPhysicsShape physicsShape { get; private set; }

        public IPhysicsShape currPhysicsShape;
        public IPhysicsShape prevPhysicsShape;

        protected virtual void PhysicsInitialize()
        {
            SyncShape();
            PhysicsGenerator.Instance.RegisterPhysicsObject(this);
        }

        private void OnDestroy()
        {
            PhysicsGenerator.Instance.UnregisterPhysicsObject(this);
        }

        private IPhysicsShape CreateShape(PhysicsShapeType physicsShapeType)
        {
            return physicsShapeType switch
            {
                PhysicsShapeType.SPHERE => new Sphere(transform),
                PhysicsShapeType.OBB => new OBB(transform),
                PhysicsShapeType.CAPSULE => new Capsule(transform)
            };
        }

        protected virtual IPhysicsShape CalculatePhysicsShape() => currPhysicsShape.ComputeSweptVolume(prevPhysicsShape);

        private void SyncShape()
        {
            if (currPhysicsShape == null ||
                prevPhysicsShape == null ||
                prevPhysicsShape.GetType() != currPhysicsShape.GetType() ||
                currPhysicsShape.ShapeType != physicsShapeType)
            {
                currPhysicsShape = CreateShape(physicsShapeType);
                prevPhysicsShape = CreateShape(physicsShapeType);
            }
            currPhysicsShape.UpdateFromTransform(transform);
            physicsShape = CalculatePhysicsShape();
            prevPhysicsShape.CopyFrom(currPhysicsShape);
        }

        private void Update()
        {
            SyncShape();
        }

        #region Gizmo
        private void OnDrawGizmos()
        {
#if UNITY_EDITOR
            if (!PhysicsGizmoToggleWindow.IsShowingGizmos()) return;

            if (Application.isPlaying == false)
            {
                SyncShape();
            }

            PhysicsGizmoDrawer.OnDrawGizmoPhysicsShape(currPhysicsShape, PhysicsGizmoToggleWindow.GetPhysicsShapeGizmoColor());

            if (!PhysicsGizmoToggleWindow.IsShowSweptGizmo() || Active == false) return;
            PhysicsGizmoDrawer.OnDrawGizmoPhysicsShape(physicsShape, PhysicsGizmoToggleWindow.GetPhysicsSweptVolumeGizmoColor());
#endif
        }
        #endregion
    }
#if UNITY_EDITOR
    public class PhysicsGizmoToggleWindow : EditorWindow
    {
        private static bool showPhysicsGizmos = true;
        private static bool showPhysicsSweptVolume = false;
        private static Color physicsShapeColor = Color.cyan;
        private static Color physicsSweptColor = Color.green;

        private const string ShowGizmoKey = "ShowPhysicsGizmos";
        private const string ShowSweptGimzoKey = "ShowPhysicsSweptGizmo";

        // Physics Gizmo Color Keys
        private const string PhysicsShapeGizmoColorR = "PhysicsShapeGizmoColorR";
        private const string PhysicsShapeGizmoColorG = "PhysicsShapeGizmoColorG";
        private const string PhysicsShapeGizmoColorB = "PhysicsShapeGizmoColorB";
        private const string PhysicsShapeGizmoColorA = "PhysicsShapeGizmoColorA";

        // Physics Swept Volume Gizmo Color Keys
        private const string PhysicsSweptGizmoColorR = "PhysicsSweptGizmoColorR";
        private const string PhysicsSweptGizmoColorG = "PhysicsSweptGizmoColorG";
        private const string PhysicsSweptGizmoColorB = "PhysicsSweptGizmoColorB";
        private const string PhysicsSweptGizmoColorA = "PhysicsSweptGizmoColorA";

        [MenuItem("Tools/Physics Gizmo Setting")]
        public static void ShowWindow()
        {
            GetWindow<PhysicsGizmoToggleWindow>("Physics Gizmo Setting");
        }

        private void OnEnable()
        {
            LoadPrefs();
        }

        private void OnGUI()
        {
            EditorGUI.BeginChangeCheck();

            showPhysicsGizmos = EditorGUILayout.Toggle("Show Physics Gizmos", showPhysicsGizmos);
            showPhysicsSweptVolume = EditorGUILayout.Toggle("Show Physics Swept Volume Gizmo", showPhysicsSweptVolume);
            physicsShapeColor = EditorGUILayout.ColorField("Physics Range Color", physicsShapeColor);
            physicsSweptColor = EditorGUILayout.ColorField("Physics Swept Volume Color", physicsSweptColor);

            if (EditorGUI.EndChangeCheck())
            {
                SavePrefs();
            }
        }

        private static void SavePrefs()
        {
            EditorPrefs.SetBool(ShowGizmoKey, showPhysicsGizmos);
            EditorPrefs.SetBool(ShowSweptGimzoKey, showPhysicsSweptVolume);

            EditorPrefs.SetFloat(PhysicsShapeGizmoColorR, physicsShapeColor.r);
            EditorPrefs.SetFloat(PhysicsShapeGizmoColorG, physicsShapeColor.g);
            EditorPrefs.SetFloat(PhysicsShapeGizmoColorB, physicsShapeColor.b);
            EditorPrefs.SetFloat(PhysicsShapeGizmoColorA, physicsShapeColor.a);

            EditorPrefs.SetFloat(PhysicsSweptGizmoColorR, physicsSweptColor.r);
            EditorPrefs.SetFloat(PhysicsSweptGizmoColorG, physicsSweptColor.g);
            EditorPrefs.SetFloat(PhysicsSweptGizmoColorB, physicsSweptColor.b);
            EditorPrefs.SetFloat(PhysicsSweptGizmoColorA, physicsSweptColor.a);
        }

        private static void LoadPrefs()
        {
            showPhysicsGizmos = EditorPrefs.GetBool(ShowGizmoKey, true);
            showPhysicsSweptVolume = EditorPrefs.GetBool(ShowSweptGimzoKey, false);

            float physicsShapeColorR = EditorPrefs.GetFloat(PhysicsShapeGizmoColorR, physicsShapeColor.r);
            float physicsShapeColorG = EditorPrefs.GetFloat(PhysicsShapeGizmoColorG, physicsShapeColor.g);
            float physicsShapeColorB = EditorPrefs.GetFloat(PhysicsShapeGizmoColorB, physicsShapeColor.b);
            float physicsShapeColorA = EditorPrefs.GetFloat(PhysicsShapeGizmoColorA, physicsShapeColor.a);
            physicsShapeColor = new Color(physicsShapeColorR, physicsShapeColorG, physicsShapeColorB, physicsShapeColorA);

            float physicsSweptColorR = EditorPrefs.GetFloat(PhysicsSweptGizmoColorR, physicsSweptColor.r);
            float physicsSweptColorG = EditorPrefs.GetFloat(PhysicsSweptGizmoColorG, physicsSweptColor.g);
            float physicsSweptColorB = EditorPrefs.GetFloat(PhysicsSweptGizmoColorB, physicsSweptColor.b);
            float physicsSweptColorA = EditorPrefs.GetFloat(PhysicsSweptGizmoColorA, physicsSweptColor.a);
            physicsSweptColor = new Color(physicsSweptColorR, physicsSweptColorG, physicsSweptColorB, physicsSweptColorA);
        }

        public static bool IsShowingGizmos()
        {
            return EditorPrefs.GetBool(ShowGizmoKey, true);
        }

        public static bool IsShowSweptGizmo()
        {
            return EditorPrefs.GetBool(ShowSweptGimzoKey, false);
        }

        public static Color GetPhysicsShapeGizmoColor()
        {
            float r = EditorPrefs.GetFloat(PhysicsShapeGizmoColorR, Color.cyan.r);
            float g = EditorPrefs.GetFloat(PhysicsShapeGizmoColorG, Color.cyan.g);
            float b = EditorPrefs.GetFloat(PhysicsShapeGizmoColorB, Color.cyan.b);
            float a = EditorPrefs.GetFloat(PhysicsShapeGizmoColorA, Color.cyan.a);
            return new Color(r, g, b, a);
        }

        public static Color GetPhysicsSweptVolumeGizmoColor()
        {
            float r = EditorPrefs.GetFloat(PhysicsSweptGizmoColorR, Color.green.r);
            float g = EditorPrefs.GetFloat(PhysicsSweptGizmoColorG, Color.green.g);
            float b = EditorPrefs.GetFloat(PhysicsSweptGizmoColorB, Color.green.b);
            float a = EditorPrefs.GetFloat(PhysicsSweptGizmoColorA, Color.green.a);
            return new Color(r, g, b, a);
        }
    }
#endif
    public class PhysicsGizmoDrawer
    {
        public static void OnDrawGizmoPhysicsShape(IPhysicsShape physicsShape, Color color)
        {
            switch (true)
            {
                case true when physicsShape is Sphere:
                    OnDrawGizmoSphere((Sphere)physicsShape, color);
                    break;

                case true when physicsShape is OBB:
                    OnDrawGizmoOBB((OBB)physicsShape, color);
                    break;

                case true when physicsShape is Capsule:
                    OnDrawGizmoCapsule((Capsule)physicsShape, color);
                    break;
            }
        }

        private static void OnDrawGizmoSphere(Sphere sphere, Color color)
        {
            Gizmos.color = color;
            Gizmos.DrawWireSphere(sphere.center, sphere.radius);
        }

        private static void OnDrawGizmoOBB(OBB oBB, Color color)
        {
            Gizmos.color = color;

            Vector3 center = oBB.center;
            Vector3 halfSize = oBB.halfSize;

            Vector3 right = oBB.axis[0];
            Vector3 up = oBB.axis[1];
            Vector3 forward = oBB.axis[2];

            Vector3[] vertices = new Vector3[8];

            vertices[0] = center + right * halfSize.x + up * halfSize.y + forward * halfSize.z;
            vertices[1] = center + right * halfSize.x + up * halfSize.y - forward * halfSize.z;
            vertices[2] = center + right * halfSize.x - up * halfSize.y + forward * halfSize.z;
            vertices[3] = center + right * halfSize.x - up * halfSize.y - forward * halfSize.z;
            vertices[4] = center - right * halfSize.x + up * halfSize.y + forward * halfSize.z;
            vertices[5] = center - right * halfSize.x + up * halfSize.y - forward * halfSize.z;
            vertices[6] = center - right * halfSize.x - up * halfSize.y + forward * halfSize.z;
            vertices[7] = center - right * halfSize.x - up * halfSize.y - forward * halfSize.z;

            // 상단면
            Gizmos.DrawLine(vertices[0], vertices[1]);
            Gizmos.DrawLine(vertices[1], vertices[3]);
            Gizmos.DrawLine(vertices[3], vertices[2]);
            Gizmos.DrawLine(vertices[2], vertices[0]);

            // 하단면
            Gizmos.DrawLine(vertices[4], vertices[5]);
            Gizmos.DrawLine(vertices[5], vertices[7]);
            Gizmos.DrawLine(vertices[7], vertices[6]);
            Gizmos.DrawLine(vertices[6], vertices[4]);

            // 수직 연결
            Gizmos.DrawLine(vertices[0], vertices[4]);
            Gizmos.DrawLine(vertices[1], vertices[5]);
            Gizmos.DrawLine(vertices[2], vertices[6]);
            Gizmos.DrawLine(vertices[3], vertices[7]);
        }

        private static void OnDrawGizmoCapsule(Capsule capsule, Color color)
        {
            Gizmos.color = color;

            Vector3 dir = math.normalize((capsule.pointB - capsule.pointA));
            float height = Vector3.Distance(capsule.pointA, capsule.pointB);

            // 양 끝 구체
            Gizmos.DrawWireSphere(capsule.pointA, capsule.radius);
            Gizmos.DrawWireSphere(capsule.pointB, capsule.radius);

            // 중간 몸통 연결 (4방향에서)
            Vector3 up = Vector3.Cross(dir, Vector3.right).normalized;
            if (up == Vector3.zero)
                up = Vector3.Cross(dir, Vector3.forward).normalized;

            Vector3 right = Vector3.Cross(dir, up).normalized;

            up *= capsule.radius;
            right *= capsule.radius;

            Vector3 pointA = capsule.pointA;
            Vector3 pointB = capsule.pointB;

            Gizmos.DrawLine(pointA + up, pointB + up);
            Gizmos.DrawLine(pointA - up, pointB - up);
            Gizmos.DrawLine(pointA + right, pointB + right);
            Gizmos.DrawLine(pointA - right, pointB - right);
        }
    }
}