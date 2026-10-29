using UnityEngine;

namespace SpaceRobot {

// Drives one limb from a Unity transform, such as a tracked XR controller.
//
// The transform supplies a target position only. MuJoCo still resolves the
// contact between the hand and whatever it touches, so no Unity colliders are
// involved and the physics remains the same as in the autonomous mission.
//
// Attach to the same object as SpaceRobotMission, set Hand to the controller
// transform, and the chosen limb follows it. Add one component per hand to
// drive two limbs at once; each instance drives only the limb it names.
[DefaultExecutionOrder(-100)]
public class SpaceRobotHandControl : MonoBehaviour {

  [Tooltip("Transform the limb follows. Typically a tracked XR controller.")]
  public Transform Hand;

  [Tooltip("Which limb the hand drives. FL 0, FR 1, RL 2, RR 3.")]
  public int Limb = Rig.FR;

  [Tooltip("Finger closure, 0 open and 1 closed. Drive from the grip trigger.")]
  [Range(0f, 1f)]
  public float Grip = 0f;

  [Tooltip("Release the limb from the rail when control is taken.")]
  public bool ReleaseRailLatch = true;

  [Tooltip("Upper limit on the force the limb can push with, in newtons. This " +
           "bounds contact force; it has little effect on free-space tracking.")]
  public double ForceLimit = 120.0;

  [Tooltip("How hard the limb is pulled toward the hand. Raise for tighter " +
           "tracking, lower for a softer, more compliant feel.")]
  public double Stiffness = Stabilization.DefaultStiffness;

  [Tooltip("Damping on that pull. Raise it alongside Stiffness to stop overshoot.")]
  public double Damping = Stabilization.DefaultDamping;

  private SpaceRobotMission _mission;
  private bool _engaged;

  private void Start() {
    _mission = GetComponent<SpaceRobotMission>();
    if (_mission == null) _mission = FindAnyObjectByType<SpaceRobotMission>();
  }

  private void FixedUpdate() {
    var controller = _mission != null ? _mission.Controller : null;
    var model = _mission != null ? _mission.Model : null;
    if (controller == null || model == null) return;

    if (Hand == null) {
      if (_engaged) { controller.ClearExternalLimb(Limb); _engaged = false; }
      return;
    }

    if (!_engaged) {
      if (ReleaseRailLatch) controller.SetRailLatch(Limb, false);
      _engaged = true;
    }

    // Unity is Y up and MuJoCo is Z up, and limb targets are given in the
    // station's frame because the station itself moves.
    Vector3 p = Hand.position;
    V3 mujocoWorld = new V3(p.x, p.z, p.y);
    V3 stationLocal = model.WorldToStationLocal(mujocoWorld);

    controller.SetExternalLimb(Limb, stationLocal, Grip, ForceLimit, Stiffness, Damping);
  }

  private void OnDisable() {
    _mission?.Controller?.ClearExternalLimb(Limb);
    _engaged = false;
  }
}
}
