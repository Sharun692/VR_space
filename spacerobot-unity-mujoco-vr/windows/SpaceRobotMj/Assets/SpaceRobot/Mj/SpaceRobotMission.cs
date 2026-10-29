using System;
using Mujoco;
using UnityEngine;

namespace SpaceRobot {

// Drives the service mission from inside MuJoCo's own step loop. MuJoCo runs
// in-process: there is no external physics server and no network stream.
public unsafe class SpaceRobotMission : MonoBehaviour {

  [Tooltip("Simulated seconds after which the mission stops advancing. Zero runs indefinitely.")]
  public double Duration = 0.0;

  [Tooltip("Off holds the robot on the rail and never starts the cap sequence, " +
           "leaving the limbs free to be driven from outside.")]
  public bool AutonomousMission = true;

  [Tooltip("Upper bound on 1 ms MuJoCo steps taken in one rendered frame. A " +
           "longer frame drops the excess rather than letting the simulation " +
           "fall ever further behind.")]
  public int MaxSubstepsPerFrame = 50;

  public const double PhysicsTimestep = 0.001;

  public string Stage => _controller?.Stage ?? "uninitialised";
  public double SimulationTime => _access == null ? 0.0 : _access.Data->time;
  public double CapAngle => _access?.CapAngle ?? 0.0;
  public ContactReport Contacts { get; private set; }

  // Exposed so that external control, such as a tracked hand, can take over a
  // limb. Both are null until the first physics step has run.
  public MissionController Controller => _controller;
  public MjModelAccess Model => _access;

  // Raised once per physics substep with the forces measured by MuJoCo, so a
  // resistive device driver receives samples at the physics rate rather than
  // the frame rate.
  public event Action<ForceSample> ForceSampleAvailable;

  public struct ForceSample {
    public double Time;
    public double CapContactForce;
    public double GripForceFL, GripForceFR, GripForceRL, GripForceRR;
    public int CapFingerContacts;
  }

  private MjScene _scene;
  private MjModelAccess _access;
  private MissionController _controller;
  private ContactClassifier _classifier;
  private Stabilization _stabilization;
  private bool _initialised;
  private double _pendingTime;

  private void Awake() {
    // The plug-in compiles the model with Time.fixedDeltaTime as its timestep
    // when MjScene starts, and every Awake runs before any Start.
    Time.fixedDeltaTime = (float)PhysicsTimestep;
  }

  private void Start() {
    _scene = MjScene.Instance;
    // Stepping from FixedUpdate advances the physics in fixed chunks that do
    // not line up with rendered frames, which shows as judder. The steps are
    // taken from Update instead, matched to each frame's duration.
    _scene.ManualStepping = true;
  }

  private void Update() {
    if (_scene == null || _scene.Model == null || _scene.Data == null) return;
    if (!_initialised) {
      // Enforced on the compiled model too, in case anything else changed
      // fixedDeltaTime before MjScene compiled it. FixedUpdate then no longer
      // carries the physics and goes back to a frame-sized interval.
      _scene.Model->opt.timestep = PhysicsTimestep;
      Time.fixedDeltaTime = 1f / 90f;
      _access = new MjModelAccess(_scene.Model, _scene.Data);
      _classifier = new ContactClassifier(_access);
      _stabilization = new Stabilization(_access);
      _stabilization.InitializeModel();
      _controller = new MissionController(_access, _stabilization);
      _initialised = true;
    }

    if (Duration > 0.0 && _access.Data->time >= Duration) return;

    _pendingTime += Time.deltaTime;
    int steps = (int)(_pendingTime / PhysicsTimestep);
    if (steps > MaxSubstepsPerFrame) {
      steps = MaxSubstepsPerFrame;
      _pendingTime = steps * PhysicsTimestep;
    }
    if (steps == 0) return;
    _pendingTime -= steps * PhysicsTimestep;

    // Advance all but the final step here. StepScene takes the last one and
    // synchronises the Unity transforms once, so the frame shows the state at
    // its own time.
    for (int i = 0; i < steps - 1; i++) {
      StepOnce();
      MujocoLib.mj_step(_access.Model, _access.Data);
    }
    StepOnce();
    _scene.StepScene();
  }

  private void StepOnce() {
    _controller.Autonomous = AutonomousMission;
    Contacts = _classifier.Measure(_access);
    _controller.Update(_access.Data->time, Contacts);
    ForceSampleAvailable?.Invoke(new ForceSample {
      Time = _access.Data->time,
      CapContactForce = Contacts.CapContactForce,
      GripForceFL = Contacts.GripForces[Rig.FL],
      GripForceFR = Contacts.GripForces[Rig.FR],
      GripForceRL = Contacts.GripForces[Rig.RL],
      GripForceRR = Contacts.GripForces[Rig.RR],
      CapFingerContacts = Contacts.CapFingerCount(Rig.FR),
    });
  }
}
}
