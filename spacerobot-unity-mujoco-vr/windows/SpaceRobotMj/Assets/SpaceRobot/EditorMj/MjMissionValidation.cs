using System;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Text;
using System.Xml;
using Mujoco;
using SpaceRobot;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using Debug = UnityEngine.Debug;

public static class MjMissionValidation {

  [MenuItem("SpaceRobot/Run in-Unity mission validation")]
  public static unsafe void Run() {
    if (!BuildSpaceRobotMjScene.ConfirmSceneReplace()) return;
    var mjcfPath = Path.GetFullPath(Path.Combine(Application.dataPath, "../tools/spacerobot_high_fidelity.xml"));
    Time.fixedDeltaTime = 0.001f;
    EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

    var xml = new XmlDocument();
    xml.Load(mjcfPath);
    new MjcfImporter().ImportXml(xml, "SpaceRobot");

    var settings = UnityEngine.Object.FindFirstObjectByType<MjGlobalSettings>();
    if (settings == null) settings = new GameObject("MjGlobalSettings").AddComponent<MjGlobalSettings>();
    settings.UseRawGameObjectNames = true;

    var scene = new GameObject("MjScene").AddComponent<MjScene>();
    scene.CreateScene();

    var m = new MjModelAccess(scene.Model, scene.Data);
    var classifier = new ContactClassifier(m);
    var stabilization = new Stabilization(m);
    stabilization.InitializeModel();
    var controller = new MissionController(m, stabilization);

    double duration = 150.0;
    int frames = 0;
    double minLatches = 99, maxCapForce = 0, maxCapFingers = 0;
    var sw = Stopwatch.StartNew();

    while (m.Data->time < duration) {
      var contacts = classifier.Measure(m);
      controller.Update(m.Data->time, contacts);
      MujocoLib.mj_step(m.Model, m.Data);

      if (frames % 50 == 0) {
        int latched = 0;
        for (int leg = 0; leg < Rig.LegCount; leg++) if (controller.RailLatched(leg)) latched++;
        minLatches = Math.Min(minLatches, latched);
        maxCapForce = Math.Max(maxCapForce, contacts.CapContactForce);
        maxCapFingers = Math.Max(maxCapFingers, contacts.CapFingerCount(Rig.FR));
      }
      frames++;
    }
    sw.Stop();

    double capAngle = m.CapAngle;
    Debug.Log($"MISSION time={m.Data->time:R}");
    Debug.Log($"MISSION final_cap_angle_rad={capAngle:R}");
    Debug.Log($"MISSION final_stage={controller.Stage}");
    Debug.Log($"MISSION regrasp_failures={controller.RegraspFailures}");
    Debug.Log($"MISSION min_active_rail_latches={minLatches}");
    Debug.Log($"MISSION max_cap_finger_contacts={maxCapFingers}");
    Debug.Log($"MISSION max_cap_contact_force_N={maxCapForce:R}");
    Debug.Log($"MISSION steps={frames} wall_s={sw.Elapsed.TotalSeconds:F1} realtime={duration / sw.Elapsed.TotalSeconds:F1}x");
    BuildSpaceRobotMjScene.Finish();
  }
}
