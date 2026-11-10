using SpaceRobot;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

// Checks the body and hand components in Play mode, the way an XR rig drives
// them: plain scene objects stand in for the headset and the two controllers.
// The head target is moved and turned, and the robot body and front limbs must
// follow it. The test objects are removed when Play mode ends.
//
// Entering Play mode reloads scripts, so the check re-attaches on load.
[InitializeOnLoad]
public static class MjPlayModeControlTest {
  private const string ScenePath = "Assets/SpaceRobot/Scenes/SpaceRobot_Service_Mj.unity";
  private const string Key = "SpaceRobot.MjPlayModeControlTest";

  private static double _start = -1;
  private static int _phase;
  private static GameObject _head, _left, _right;
  private static Vector3 _headStart;

  static MjPlayModeControlTest() {
    if (SessionState.GetBool(Key, false)) EditorApplication.update += Tick;
  }

  [MenuItem("SpaceRobot/Test body and hands in Play mode")]
  public static void Run() {
    if (EditorApplication.isPlaying) {
      Debug.LogWarning("PLAYTEST stop Play mode before running this check.");
      return;
    }
    if (!BuildSpaceRobotMjScene.ConfirmSceneReplace()) return;
    EditorSceneManager.OpenScene(ScenePath);
    SessionState.SetBool(Key, true);
    _start = -1;
    EditorApplication.update -= Tick;
    EditorApplication.update += Tick;
    Debug.Log("PLAYTEST started; Play mode runs for about 15 s.");
    // Entering Play mode while the editor is still indexing the project, just
    // after it opens, makes Unity's own search indexer log an error. A short
    // wait lets it finish first.
    double enterAt = EditorApplication.timeSinceStartup + 5.0;
    void EnterWhenIdle() {
      if (EditorApplication.timeSinceStartup < enterAt || EditorApplication.isCompiling
          || EditorApplication.isUpdating) return;
      EditorApplication.update -= EnterWhenIdle;
      EditorApplication.EnterPlaymode();
    }
    EditorApplication.update += EnterWhenIdle;
  }

  private static void Tick() {
    if (!EditorApplication.isPlaying) return;
    if (_start < 0) { _start = EditorApplication.timeSinceStartup; _phase = 0; }
    double t = EditorApplication.timeSinceStartup - _start;

    var mission = Object.FindAnyObjectByType<SpaceRobotMission>();
    if (mission == null) return;
    var body = mission.GetComponent<SpaceRobotBodyControl>();
    var hand = mission.GetComponent<SpaceRobotHandControl>();
    var robot = GameObject.Find("robot").transform;

    if (_phase == 0 && t > 4.0 && mission.Controller != null) {
      // Head on the body's current pose; hands 10 cm above the front grips.
      _head = new GameObject("Test head");
      _head.transform.SetPositionAndRotation(robot.position, robot.rotation);
      _headStart = _head.transform.position;
      _left = new GameObject("Test left hand");
      _right = new GameObject("Test right hand");
      _left.transform.position = GameObject.Find("FL_grasp_site").transform.position + Vector3.up * 0.10f;
      _right.transform.position = GameObject.Find("FR_grasp_site").transform.position + Vector3.up * 0.10f;

      body.Body = _head.transform;
      hand.Hand = _right.transform;
      hand.Limb = Rig.FR;
      var second = mission.gameObject.AddComponent<SpaceRobotHandControl>();
      second.Hand = _left.transform;
      second.Limb = Rig.FL;
      _phase = 1;
    } else if (_phase == 1 && t > 5.0) {
      // The head moves 8 cm and rises 6 cm, and turns 10 degrees.
      _head.transform.position = _headStart + new Vector3(0.08f, 0.06f, 0.0f);
      _head.transform.rotation = Quaternion.AngleAxis(10f, Vector3.up) * _head.transform.rotation;
      _phase = 2;
    } else if (_phase == 2 && t > 13.0) {
      float bodyPosition = Vector3.Distance(robot.position, _head.transform.position);
      float bodyAngle = Quaternion.Angle(robot.rotation, _head.transform.rotation);
      float handError = Mathf.Max(
          Vector3.Distance(GameObject.Find("FR_grasp_site").transform.position, _right.transform.position),
          Vector3.Distance(GameObject.Find("FL_grasp_site").transform.position, _left.transform.position));
      bool missionOff = !mission.AutonomousMission;

      Debug.Log($"PLAYTEST body position_error={bodyPosition:F4} m  angle_error={bodyAngle:F2} deg");
      Debug.Log($"PLAYTEST hands max_error={handError:F4} m");
      Debug.Log($"PLAYTEST autonomous_mission_switched_off={missionOff}");
      bool pass = bodyPosition < 0.02f && bodyAngle < 2f && handError < 0.02f && missionOff;
      Debug.Log($"PLAYTEST verdict={(pass ? "BODY AND HANDS FOLLOW" : "FAILED")}");

      _phase = 3;
      SessionState.SetBool(Key, false);
      EditorApplication.update -= Tick;
      // Leaving Play mode discards the test objects and the extra component.
      EditorApplication.ExitPlaymode();
    }
  }
}
