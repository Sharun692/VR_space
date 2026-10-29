using UnityEngine;

namespace SpaceRobot {

// Orbit camera. Left-drag orbits, right- or middle-drag pans, the wheel zooms
// and F recentres on the robot.
public class SpaceRobotObserverCamera : MonoBehaviour {
  public Transform Target;
  public Vector3 Offset = new Vector3(-1.8f, 2.8f, -4.8f);
  public float OrbitSpeed = 0.25f;
  public float PanSpeed = 0.0035f;
  public float ZoomSpeed = 0.6f;

  private Vector3 _pan;
  private Vector3 _lastMouse;
  private float _yaw, _pitch, _distance;

  private void Start() => SetView(Offset);

  private void SetView(Vector3 viewOffset) {
    _distance = viewOffset.magnitude;
    _yaw = Mathf.Atan2(viewOffset.x, viewOffset.z) * Mathf.Rad2Deg;
    _pitch = Mathf.Asin(viewOffset.y / _distance) * Mathf.Rad2Deg;
    _pan = Vector3.zero;
  }

  private void LateUpdate() {
    Vector3 mouse = Input.mousePosition;
    Vector3 delta = mouse - _lastMouse;
    _lastMouse = mouse;

    if (Input.GetMouseButton(0)) {
      _yaw += delta.x * OrbitSpeed;
      _pitch = Mathf.Clamp(_pitch - delta.y * OrbitSpeed, -85f, 85f);
    } else if (Input.GetMouseButton(1) || Input.GetMouseButton(2)) {
      _pan -= transform.right * delta.x * PanSpeed * _distance
            + transform.up * delta.y * PanSpeed * _distance;
    }

    float wheel = Input.mouseScrollDelta.y;
    if (Mathf.Abs(wheel) > 0f) _distance = Mathf.Clamp(_distance - wheel * ZoomSpeed, 0.6f, 60f);
    if (Input.GetKeyDown(KeyCode.F)) SetView(Offset);

    Vector3 focus = (Target != null ? Target.position : Vector3.zero) + _pan;
    Quaternion rotation = Quaternion.Euler(_pitch, _yaw, 0f);
    transform.position = focus + rotation * new Vector3(0f, 0f, -_distance);
    transform.rotation = Quaternion.LookRotation(focus - transform.position, Vector3.up);
  }
}
}
