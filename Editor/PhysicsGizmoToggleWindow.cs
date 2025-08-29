using UnityEditor;
using UnityEngine;


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