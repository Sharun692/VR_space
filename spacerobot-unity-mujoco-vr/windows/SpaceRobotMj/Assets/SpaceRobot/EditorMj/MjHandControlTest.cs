using System;
using System.IO;
using System.Xml;
using Mujoco;
using SpaceRobot;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using Debug = UnityEngine.Debug;

// Checks that a limb handed to external control follows a supplied target,
// with the contact still resolved by MuJoCo.
public static class MjHandControlTest {


  [MenuItem("SpaceRobot/Test external limb control")]
  public static unsafe void Run() {
    if (!BuildSpaceRobotMjScene.ConfirmSceneReplace()) return;
    Time.fixedDeltaTime = 0.001f;
    EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
    var xml = new XmlDocument();
    xml.Load(Path.GetFullPath(Path.Combine(Application.dataPath, "../tools/spacerobot_high_fidelity.xml")));
    new MjcfImporter().ImportXml(xml, "SpaceRobot");
    var settings = UnityEngine.Object.FindFirstObjectByType<MjGlobalSettings>()
                   ?? new GameObject("MjGlobalSettings").AddComponent<MjGlobalSettings>();
    settings.UseRawGameObjectNames = true;
    var scene = new GameObject("MjScene").AddComponent<MjScene>();
    scene.CreateScene();

    var m = new MjModelAccess(scene.Model, scene.Data);
    var classifier = new ContactClassifier(m);
    var stabilization = new Stabilization(m);
    stabilization.InitializeModel();
    var controller = new MissionController(m, stabilization);

    // Settle on the rail under the autonomous controller.
    while (m.Data->time < 2.5) {
      controller.Update(m.Data->time, classifier.Measure(m));
      MujocoLib.mj_step(m.Model, m.Data);
    }

    // Two limbs at once, as a pair of tracked hands would drive them.
    int[] legs = { Rig.FR, Rig.FL };
    var site = new int[legs.Length];
    var start = new V3[legs.Length];
    var target = new V3[legs.Length];

    controller.Autonomous = false;
    for (int i = 0; i < legs.Length; i++) {
      site[i] = m.GraspSite[legs[i]];
      start[i] = m.WorldToStationLocal(m.SitePos(site[i]));
      target[i] = start[i] + new V3(0.0, 0.0, 0.25);   // 25 cm off the rail
      controller.SetRailLatch(legs[i], false);
      controller.SetExternalLimb(legs[i], target[i], 0.0, 120.0);
    }

    double deadline = m.Data->time + 4.0;
    while (m.Data->time < deadline) {
      controller.Update(m.Data->time, classifier.Measure(m));
      MujocoLib.mj_step(m.Model, m.Data);
    }

    bool allFollowed = true;
    for (int i = 0; i < legs.Length; i++) {
      V3 reached = m.WorldToStationLocal(m.SitePos(site[i]));
      double before = (target[i] - start[i]).Norm;
      double after = (target[i] - reached).Norm;
      if (after >= before * 0.25) allFollowed = false;
      Debug.Log($"HAND limb={Rig.Legs[legs[i]]} distance_before={before:F4} m  distance_after={after:F4} m");
    }
    Debug.Log($"HAND stage={controller.Stage}");
    Debug.Log($"HAND limbs_driven={legs.Length}");
    Debug.Log($"HAND verdict={(allFollowed ? "ALL FOLLOW TARGET" : "DID NOT FOLLOW")}");
    BuildSpaceRobotMjScene.Finish();
  }
}
