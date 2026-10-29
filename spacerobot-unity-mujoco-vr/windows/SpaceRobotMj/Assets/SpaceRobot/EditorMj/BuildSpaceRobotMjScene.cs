using System;
using System.IO;
using System.Xml;
using Mujoco;
using SpaceRobot;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEditor.SceneManagement;
using UnityEngine;

public static class BuildSpaceRobotMjScene {

  private const string ScenePath = "Assets/SpaceRobot/Scenes/SpaceRobot_Service_Mj.unity";

  // Asks to save unsaved scene changes before a command replaces the scene.
  // False when the user cancels.
  internal static bool ConfirmSceneReplace() =>
      EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo();

  // Command-line runs (-batchmode -executeMethod) end by quitting. From the
  // menu the editor stays open and returns to the service scene.
  internal static void Finish() {
    if (Application.isBatchMode) EditorApplication.Exit(0);
    else EditorSceneManager.OpenScene(ScenePath);
  }

  [MenuItem("SpaceRobot/Build in-Unity MuJoCo scene")]
  public static void Build() {
    if (!ConfirmSceneReplace()) return;
    EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

    // MuJoCo integrates at 1 ms; Unity's default 20 ms would silently replace it.
    Time.fixedDeltaTime = 0.001f;

    var mjcfPath = Path.GetFullPath(Path.Combine(Application.dataPath, "../tools/spacerobot_high_fidelity.xml"));
    var xml = new XmlDocument();
    xml.Load(mjcfPath);
    new MjcfImporter().ImportXml(xml, "SpaceRobot");

    // Without this the plug-in renames every MuJoCo element (robot -> robot_0).
    var settings = UnityEngine.Object.FindFirstObjectByType<MjGlobalSettings>();
    if (settings == null) settings = new GameObject("MjGlobalSettings").AddComponent<MjGlobalSettings>();
    settings.UseRawGameObjectNames = true;

    // MjScene is deliberately not added here: the plug-in creates it as the
    // imported components come alive, and a second instance is an error.
    var sceneObject = new GameObject("Mission");
    sceneObject.AddComponent<SpaceRobotMission>();
    sceneObject.AddComponent<SpaceRobotHaptics>();
    sceneObject.AddComponent<SpaceRobotCapture>();
    // Inert until a Hand transform is assigned.
    sceneObject.AddComponent<SpaceRobotHandControl>();

    ApplyMujocoColours();

    var cameraObject = new GameObject("Observer camera");
    var camera = cameraObject.AddComponent<Camera>();
    camera.clearFlags = CameraClearFlags.SolidColor;
    camera.backgroundColor = new Color(0.006f, 0.011f, 0.032f);
    camera.fieldOfView = 58f;
    camera.farClipPlane = 400f;
    cameraObject.tag = "MainCamera";
    var observer = cameraObject.AddComponent<SpaceRobotObserverCamera>();
    var robot = GameObject.Find("robot");
    if (robot != null) observer.Target = robot.transform;

    var lightObject = new GameObject("Sun light");
    var light = lightObject.AddComponent<Light>();
    light.type = LightType.Directional;
    light.intensity = 1.4f;
    lightObject.transform.rotation = Quaternion.Euler(35f, -30f, 0f);
    RenderSettings.ambientLight = new Color(0.22f, 0.26f, 0.34f);

    Directory.CreateDirectory(Path.Combine(Application.dataPath, "SpaceRobot/Scenes"));
    if (!EditorSceneManager.SaveScene(EditorSceneManager.GetActiveScene(), ScenePath)) {
      throw new Exception("Failed to save the scene.");
    }
    AssetDatabase.SaveAssets();
    Debug.Log("MJ scene saved: " + ScenePath);
  }

  // The importer gives every geom the same default material. MuJoCo's own rgba
  // values are read back from the compiled model so the scene matches the
  // simulation it is showing.
  private static unsafe void ApplyMujocoColours() {
    Directory.CreateDirectory(Path.Combine(Application.dataPath, "SpaceRobot/Materials"));
    AssetDatabase.Refresh();

    // Compile the ORIGINAL MJCF: the plug-in does not round-trip rgba through
    // its components, so the regenerated model carries default colours only.
    var mjcfPath = Path.GetFullPath(
        Path.Combine(Application.dataPath, "../tools/spacerobot_high_fidelity.xml"));
    var source = MjEngineTool.LoadModelFromString(File.ReadAllText(mjcfPath));
    try {
      var materials = new System.Collections.Generic.Dictionary<Color, Material>();
      foreach (var geom in UnityEngine.Object.FindObjectsByType<MjGeom>(FindObjectsSortMode.None)) {
        int id = MujocoLib.mj_name2id(
            source, (int)MujocoLib.mjtObj.mjOBJ_GEOM, geom.gameObject.name);
        if (id < 0) continue;
        // Colour comes from the geom's material when it has one; MuJoCo falls
        // back to the geom's own rgba otherwise.
        int matid = source->geom_matid[id];
        var rgba = matid >= 0 ? source->mat_rgba + matid * 4 : source->geom_rgba + id * 4;
        var colour = new Color(rgba[0], rgba[1], rgba[2], rgba[3]);
        var renderer = geom.GetComponent<Renderer>();
        if (renderer == null) continue;
        if (!materials.TryGetValue(colour, out var material)) {
          material = new Material(Shader.Find("Standard")) { color = colour };
          if (colour.a < 0.999f) {
            material.SetFloat("_Mode", 3);
            material.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
            material.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
            material.SetInt("_ZWrite", 0);
            material.EnableKeyword("_ALPHABLEND_ON");
            material.renderQueue = 3000;
          }
          AssetDatabase.CreateAsset(
              material, $"Assets/SpaceRobot/Materials/mj_{materials.Count:D2}.mat");
          materials[colour] = material;
        }
        renderer.sharedMaterial = material;
      }
      Debug.Log($"MJCOLOURS applied {materials.Count} distinct materials");
    } finally {
      MujocoLib.mj_deleteModel(source);
    }
  }

  [MenuItem("SpaceRobot/Build in-Unity MuJoCo Windows player")]
  public static void BuildWindows() {
    if (!ConfirmSceneReplace()) return;
    Build();
    var output = Path.GetFullPath(Path.Combine(
        Application.dataPath, "../../SpaceRobot_Player/SpaceRobot_Service.exe"));
    Directory.CreateDirectory(Path.GetDirectoryName(output));
    var report = BuildPipeline.BuildPlayer(new BuildPlayerOptions {
      scenes = new[] { ScenePath },
      locationPathName = output,
      target = BuildTarget.StandaloneWindows64,
      options = BuildOptions.None
    });
    if (report.summary.result != BuildResult.Succeeded) {
      throw new Exception("Windows player build failed: " + report.summary.result);
    }
    Debug.Log("MJ Windows player built: " + output);
    if (!Application.isBatchMode) EditorUtility.RevealInFinder(output);
    Finish();
  }
}
