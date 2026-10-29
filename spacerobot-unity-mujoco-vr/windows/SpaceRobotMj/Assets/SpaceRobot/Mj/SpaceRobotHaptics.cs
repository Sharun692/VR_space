using UnityEngine;
using UnityEngine.XR;
using System.Collections.Generic;

namespace SpaceRobot {

// Maps a measured contact force onto controller vibration. Force samples arrive
// at the physics rate, so a resistive device can be driven from the same event
// without resampling.
public class SpaceRobotHaptics : MonoBehaviour {

  public enum Source {
    // Everything the limb's fingers are touching, rail and cap alike. This is
    // what is felt while gripping the rail or walking.
    LimbGrip,
    // Contact with the service cap only.
    CapContact,
  }

  public XRNode Hand = XRNode.RightHand;

  [Tooltip("Which measured force drives the vibration.")]
  public Source From = Source.LimbGrip;

  [Tooltip("Limb whose grip force is used, when From is LimbGrip. 0 FL, 1 FR, 2 RL, 3 RR.")]
  public int Limb = Rig.FR;

  [Tooltip("Force, in newtons, that corresponds to full amplitude.")]
  public double FullScaleForce = 180.0;

  [Tooltip("Force below which nothing is sent, in newtons.")]
  public double Threshold = 2.0;

  [Tooltip("Minimum interval between pulses, in seconds.")]
  public double PulseInterval = 0.05;

  private SpaceRobotMission _mission;
  private double _nextPulse;

  private void Start() {
    _mission = FindAnyObjectByType<SpaceRobotMission>();
    if (_mission != null) _mission.ForceSampleAvailable += OnForceSample;
  }

  private void OnDisable() {
    if (_mission != null) _mission.ForceSampleAvailable -= OnForceSample;
  }

  private double ForceFrom(SpaceRobotMission.ForceSample sample) {
    if (From == Source.CapContact) return sample.CapContactForce;
    switch (Limb) {
      case Rig.FL: return sample.GripForceFL;
      case Rig.RL: return sample.GripForceRL;
      case Rig.RR: return sample.GripForceRR;
      default:     return sample.GripForceFR;
    }
  }

  private void OnForceSample(SpaceRobotMission.ForceSample sample) {
    double force = ForceFrom(sample);
    if (force < Threshold || sample.Time < _nextPulse) return;
    _nextPulse = sample.Time + PulseInterval;

    float amplitude = (float)MjMath.Clamp(force / FullScaleForce, 0.0, 1.0);
    var devices = new List<InputDevice>();
    InputDevices.GetDevicesAtXRNode(Hand, devices);
    foreach (var device in devices) {
      if (device.TryGetHapticCapabilities(out var capabilities) && capabilities.supportsImpulse) {
        device.SendHapticImpulse(0, amplitude, (float)PulseInterval);
      }
    }
  }
}
}
