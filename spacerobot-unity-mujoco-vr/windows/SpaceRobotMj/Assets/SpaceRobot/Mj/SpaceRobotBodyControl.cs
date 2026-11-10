using UnityEngine;

namespace SpaceRobot {

// Drives the robot's body from a Unity transform, such as a tracked headset.
//
// The body is pulled toward the transform's position and rotation, and the
// legs still latched to the rail bend to carry it there. MuJoCo resolves all
// contact, as for the limbs. The body stops where those legs run out of reach;
// OutOfReach reports when it has.
//
// Attach to the same object as SpaceRobotMission and set Body. Autonomous
// Mission is switched off when the body is first driven. It combines with SpaceRobotHandControl, for example
// the head on the body and the hands on the two front limbs.
[DefaultExecutionOrder(-100)]
public class SpaceRobotBodyControl : MonoBehaviour {

  [Tooltip("Transform the body follows, in position and rotation. The robot " +
           "body takes the same world pose as this transform.")]
  public Transform Body;

  [Tooltip("True when the body has come to rest short of the target, because " +
           "the legs on the rail cannot reach further. Read only.")]
  public bool OutOfReach;

  [Tooltip("Distance from the body to the target, in metres. Read only.")]
  public float PositionError;

  [Tooltip("Angle between the body and the target, in degrees. Read only.")]
  public float AngleError;

  private SpaceRobotMission _mission;

  private void Start() {
    _mission = GetComponent<SpaceRobotMission>();
    if (_mission == null) _mission = FindAnyObjectByType<SpaceRobotMission>();
  }

  private void FixedUpdate() {
    var controller = _mission != null ? _mission.Controller : null;
    var model = _mission != null ? _mission.Model : null;
    if (controller == null || model == null) return;

    if (Body == null) {
      controller.ClearExternalBody();
      OutOfReach = false;
      return;
    }

    // The mission's walking would also move the body, so it is switched off.
    if (_mission.AutonomousMission) {
      _mission.AutonomousMission = false;
      Debug.Log("SpaceRobotBodyControl: Autonomous Mission switched off while the body is driven.");
    }

    ToStationFrame(model, Body, out V3 position, out M3 rotation);
    controller.SetExternalBody(position, rotation);

    OutOfReach = controller.BodyOutOfReach;
    PositionError = (float)controller.BodyPositionError;
    AngleError = (float)(controller.BodyAngleError * Mathf.Rad2Deg);
  }

  // A Unity transform's world pose as a body target relative to the station.
  // Unity is Y up and MuJoCo is Z up: the two frames differ by swapping the
  // second and third axes, for positions and for rotation matrices alike.
  public static void ToStationFrame(MjModelAccess model, Transform t, out V3 position, out M3 rotation) {
    Vector3 p = t.position;
    Matrix4x4 r = Matrix4x4.Rotate(t.rotation);
    M3 rotationWorld = new M3 {
      m00 = r.m00, m01 = r.m02, m02 = r.m01,
      m10 = r.m20, m11 = r.m22, m12 = r.m21,
      m20 = r.m10, m21 = r.m12, m22 = r.m11,
    };
    position = model.WorldToStationLocal(new V3(p.x, p.z, p.y));
    rotation = model.StationMat.T * rotationWorld;
  }

  private void OnDisable() {
    _mission?.Controller?.ClearExternalBody();
    OutOfReach = false;
  }
}
}
