using System;
using System.IO;
using System.Xml;
using Mujoco;
using SpaceRobot;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using Debug = UnityEngine.Debug;

// Checks that the body follows an external pose while the two front limbs are
// driven by hand and the rear limbs hold the rail, then that a target beyond
// the legs' reach leaves the body at rest at the limit rather than unstable.
public static class MjBodyControlTest {

  [MenuItem("SpaceRobot/Test body control")]
  public static unsafe void Run() {
    if (!BuildSpaceRobotMjScene.ConfirmSceneReplace()) return;
    Time.fixedDeltaTime = 0.001f;
    EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
    var xml = new XmlDocument();
    xml.Load(Path.GetFullPath(Path.Combine(Application.dataPath, "../tools/spacerobot_high_fidelity.xml")));
    new MjcfImporter().ImportXml(xml, "SpaceRobot");
    var settings = UnityEngine.Object.FindAnyObjectByType<MjGlobalSettings>()
                   ?? new GameObject("MjGlobalSettings").AddComponent<MjGlobalSettings>();
    settings.UseRawGameObjectNames = true;
    var scene = new GameObject("MjScene").AddComponent<MjScene>();
    scene.CreateScene();

    var m = new MjModelAccess(scene.Model, scene.Data);
    var classifier = new ContactClassifier(m);
    var stabilization = new Stabilization(m);
    stabilization.InitializeModel();
    var controller = new MissionController(m, stabilization);
    bool stable = true;

    void RunFor(double seconds) {
      double end = m.Data->time + seconds;
      while (m.Data->time < end) {
        controller.Update(m.Data->time, classifier.Measure(m));
        MujocoLib.mj_step(m.Model, m.Data);
        if (m.Data->warning3.number > 0 || m.Data->warning4.number > 0 || m.Data->warning5.number > 0) {
          stable = false;
        }
      }
    }

    // Settle on the rail under the autonomous controller, then take over.
    RunFor(2.5);
    controller.Autonomous = false;
    V3 startPosition = m.WorldToStationLocal(m.RobotPos);
    M3 startRotation = m.StationMat.T * m.RobotMat;

    // 0. The component's conversion from a Unity transform. The robot body's
    // own GameObject, synced from MuJoCo, converted back must give the pose
    // MuJoCo holds for the body.
    MujocoLib.mj_kinematics(m.Model, m.Data);
    scene.SyncUnityToMjState();
    var robotObject = GameObject.Find("robot");
    SpaceRobotBodyControl.ToStationFrame(m, robotObject.transform, out V3 convertedPosition, out M3 convertedRotation);
    double conversionPosition = (convertedPosition - startPosition).Norm;
    M3 conversionDelta = convertedRotation * startRotation.T;
    double conversionAngle = Math.Acos(MjMath.Clamp((conversionDelta.Trace - 1.0) / 2.0, -1.0, 1.0)) * 180.0 / Math.PI;
    bool converted = conversionPosition < 0.001 && conversionAngle < 0.1;
    Debug.Log($"BODY transform_conversion position_error={conversionPosition:F5} m  angle_error={conversionAngle:F3} deg");

    // 1. Reach limit, body only, all four feet on the rail: a target 1.5 m
    // above the rail, far beyond what the legs can reach.
    controller.SetExternalBody(startPosition + new V3(0.0, 0.0, 1.5), startRotation);
    RunFor(6.0);
    bool limitReported = controller.BodyOutOfReach;
    double heldHeight = m.WorldToStationLocal(m.RobotPos).z - startPosition.z;
    bool allHeldAtLimit = true;
    for (int leg = 0; leg < Rig.LegCount; leg++) allHeldAtLimit &= controller.RailLatched(leg);
    Debug.Log($"BODY limit out_of_reach={limitReported}  body_raised={heldHeight:F3} m  all_limbs_latched={allHeldAtLimit}");

    controller.SetExternalBody(startPosition, startRotation);
    RunFor(8.0);
    double returnPosition = controller.BodyPositionError;
    double returnAngle = controller.BodyAngleError * 180.0 / Math.PI;
    bool returned = returnPosition < 0.02 && returnAngle < 2.0 && !controller.BodyOutOfReach;
    Debug.Log($"BODY return position_error={returnPosition:F4} m  angle_error={returnAngle:F2} deg");

    // 2. The demo: hands lift the two front limbs 10 cm off the rail, while
    // the head moves the body 10 cm forward, 5 cm sideways and 8 cm up and
    // turns it 10 degrees. The rear limbs stay on the rail.
    int[] hands = { Rig.FL, Rig.FR };
    var handTarget = new V3[hands.Length];
    for (int i = 0; i < hands.Length; i++) {
      handTarget[i] = m.WorldToStationLocal(m.SitePos(m.GraspSite[hands[i]])) + new V3(0.0, 0.0, 0.10);
      controller.SetRailLatch(hands[i], false);
      controller.SetExternalLimb(hands[i], handTarget[i], 0.0);
    }
    controller.SetExternalBody(startPosition + new V3(0.10, 0.05, 0.08),
                               M3.RotZ(10.0 * Math.PI / 180.0) * startRotation);
    RunFor(6.0);

    double followPosition = controller.BodyPositionError;
    double followAngle = controller.BodyAngleError * 180.0 / Math.PI;
    double handError = MaxHandError();
    bool rearHeld = controller.RailLatched(Rig.RL) && controller.RailLatched(Rig.RR);
    bool followed = followPosition < 0.02 && followAngle < 2.0 && handError < 0.02 && rearHeld
                    && !controller.BodyOutOfReach;
    Debug.Log($"BODY follow position_error={followPosition:F4} m  angle_error={followAngle:F2} deg  out_of_reach={controller.BodyOutOfReach}");
    Debug.Log($"BODY hands max_error={handError:F4} m  rear_limbs_latched={rearHeld}");

    // 3. The body pulled out of reach while the hands are held still, then
    // brought back. The hands must come back to their targets on their own,
    // without catching on the rail.
    controller.SetExternalBody(startPosition + new V3(0.0, 0.0, 1.5), startRotation);
    RunFor(6.0);
    controller.SetExternalBody(startPosition, startRotation);
    RunFor(8.0);
    double recoverPosition = controller.BodyPositionError;
    double recoverAngle = controller.BodyAngleError * 180.0 / Math.PI;
    double recoverHand = MaxHandError();
    bool recovered = recoverPosition < 0.02 && recoverAngle < 2.0 && recoverHand < 0.02;
    Debug.Log($"BODY hands_after_limit body_position_error={recoverPosition:F4} m  body_angle_error={recoverAngle:F2} deg  hands_max_error={recoverHand:F4} m");
    Debug.Log($"BODY physics_stable={stable}");

    bool pass = converted && limitReported && allHeldAtLimit && returned && followed && recovered && stable;
    Debug.Log($"BODY verdict={(pass ? "BODY FOLLOWS TARGET" : "FAILED")}");
    BuildSpaceRobotMjScene.Finish();

    double MaxHandError() {
      double worst = 0.0;
      for (int i = 0; i < hands.Length; i++) {
        V3 at = m.WorldToStationLocal(m.SitePos(m.GraspSite[hands[i]]));
        worst = Math.Max(worst, (handTarget[i] - at).Norm);
      }
      return worst;
    }
  }
}