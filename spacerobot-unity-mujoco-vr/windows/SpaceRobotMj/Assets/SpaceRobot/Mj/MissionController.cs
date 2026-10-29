using System;
using Mujoco;

namespace SpaceRobot {

// The autonomous cap-service mission: walk the rail, then release the
// front-right limb and operate the cap. Allocation-free once constructed.
public unsafe class MissionController {
  private readonly MjModelAccess _m;
  private readonly Stabilization _stabilization;

  private readonly V3[] _anchors = new V3[Rig.LegCount];
  private readonly V3[] _targets = new V3[Rig.LegCount];
  private readonly double[] _closures = new double[Rig.LegCount];
  private readonly double[] _twists = new double[Rig.LegCount];
  private readonly double[] _limb = new double[6];
  private readonly double[] _fingers = new double[8];

  private int _stepIndex = -1;
  private double _stepStart = Rig.SettleTime;
  private int _swingLeg = -1;
  private V3 _swingStart, _swingEnd;
  private bool _swingTargetMoved;
  private bool _manipulating;
  private double _manipulationStart;

  // Any limb can be taken over from outside. The mission stops commanding it and
  // drives it to the supplied target instead; MuJoCo still resolves the contact.
  // Limbs are independent, so two hands can drive two limbs at the same time.
  private readonly bool[] _externalActive = new bool[Rig.LegCount];
  private readonly V3[] _externalTarget = new V3[Rig.LegCount];
  private readonly double[] _externalClosure = new double[Rig.LegCount];
  private readonly double[] _externalForceLimit = new double[Rig.LegCount];
  private readonly double[] _externalStiffness = new double[Rig.LegCount];
  private readonly double[] _externalDamping = new double[Rig.LegCount];

  public bool IsExternal(int leg) => _externalActive[leg];

  // With this off, the robot holds its grip on the rail and the cap sequence
  // never starts. Station keeping and external limb control still run, which
  // is the mode to use when the robot is driven by hand.
  public bool Autonomous { get; set; } = true;

  public void SetExternalLimb(int leg, V3 stationLocalTarget, double closure,
                              double forceLimit = 120.0,
                              double stiffness = Stabilization.DefaultStiffness,
                              double damping = Stabilization.DefaultDamping) {
    _externalActive[leg] = true;
    _externalTarget[leg] = stationLocalTarget;
    _externalClosure[leg] = MjMath.Clamp(closure, 0.0, 1.0);
    _externalForceLimit[leg] = forceLimit;
    _externalStiffness[leg] = stiffness;
    _externalDamping[leg] = damping;
  }

  public void ClearExternalLimb(int leg) => _externalActive[leg] = false;

  public void ClearAllExternalLimbs() {
    for (int leg = 0; leg < Rig.LegCount; leg++) _externalActive[leg] = false;
  }

  public string Stage { get; private set; } = "initial-grasp-settle";
  public int RegraspFailures { get; private set; }
  public bool CapGraspVerified { get; private set; }

  public MissionController(MjModelAccess m, Stabilization stabilization) {
    _m = m;
    _stabilization = stabilization;
    for (int leg = 0; leg < Rig.LegCount; leg++) {
      _anchors[leg] = Rig.InitialAnchors[leg];
      m.SetModelSitePos(m.RailTargetSite[leg], _anchors[leg]);
    }
  }

  public bool RailLatched(int leg) => _m.EqActive(_m.RailLatchEq[leg]);
  public void SetRailLatch(int leg, bool active) => _m.SetEqActive(_m.RailLatchEq[leg], active);
  public bool CapLatched => _m.EqActive(_m.CapLatchEq);
  public void SetCapLatch(bool active) => _m.SetEqActive(_m.CapLatchEq, active);

  private void ContactedRailPoint(int leg, out V3 local, out V3 railCenter) {
    local = _m.StationMat.T * (_m.SitePos(_m.GraspSite[leg]) - _m.StationPos);
    railCenter = new V3(local.x, _swingEnd.y, 0.58);
  }

  private void StartNextStep(double now) {
    _stepIndex += 1;
    if (_stepIndex >= Rig.LocomotionSteps) {
      _swingLeg = -1;
      _manipulating = true;
      _manipulationStart = now + 1.5;
      Stage = "service-settle";
      return;
    }
    _swingLeg = Rig.StepOrder[_stepIndex % Rig.StepOrder.Length];
    _swingStart = _anchors[_swingLeg];
    _swingEnd = _swingStart + new V3(Rig.Stride, 0.0, 0.0);
    _swingTargetMoved = false;
    _stepStart = now;
  }

  private void ResetToAnchors() {
    for (int leg = 0; leg < Rig.LegCount; leg++) {
      _targets[leg] = _anchors[leg];
      _closures[leg] = 1.0;
      _twists[leg] = 0.0;
    }
  }

  private V3 MeanTargets() {
    var sum = new V3(0, 0, 0);
    for (int leg = 0; leg < Rig.LegCount; leg++) sum = sum + _targets[leg];
    return 0.25 * sum;
  }

  private V3 MeanAnchors() {
    var sum = new V3(0, 0, 0);
    for (int leg = 0; leg < Rig.LegCount; leg++) sum = sum + _anchors[leg];
    return 0.25 * sum;
  }

  private V3 LocomotionTargets(double now, ContactReport contacts) {
    if (_stepIndex < 0) StartNextStep(now);
    double phase = Math.Min(1.0, Math.Max(0.0, (now - _stepStart) / Rig.StepPeriod));
    int leg = _swingLeg;
    ResetToAnchors();

    if (phase < 0.18) {
      _closures[leg] = 1.0 - MjMath.Smoothstep(phase / 0.18);
      Stage = "open";
    } else {
      SetRailLatch(leg, false);
      if (!_swingTargetMoved) {
        _m.SetModelSitePos(_m.RailTargetSite[leg], _swingEnd);
        _swingTargetMoved = true;
      }
      if (phase < 0.78) {
        double swing = MjMath.Smoothstep((phase - 0.18) / 0.60);
        V3 p = (1.0 - swing) * _swingStart + swing * _swingEnd;
        p.z += Rig.SwingClearance * Math.Sin(Math.PI * swing);
        _targets[leg] = p;
        _closures[leg] = 0.0;
        Stage = "swing";
      } else if (phase < 0.90) {
        _targets[leg] = _swingEnd;
        _closures[leg] = 0.0;
        Stage = "approach";
      } else {
        _targets[leg] = _swingEnd;
        _closures[leg] = MjMath.Smoothstep((phase - 0.90) / 0.10);
        Stage = "wrap";
        if (contacts.RailFingerCount(leg) >= 2) {
          ContactedRailPoint(leg, out V3 contactPose, out V3 railCenter);
          if (railCenter.x - _swingStart.x >= 0.30) {
            _swingEnd = railCenter;
            _m.SetModelSitePos(_m.RailTargetSite[leg], contactPose);
            SetRailLatch(leg, true);
          }
        }
      }
    }

    V3 baseTarget = MeanTargets() + new V3(0.0, 0.0, 0.60);
    if (phase >= 1.0) {
      if (RailLatched(leg)) {
        _anchors[leg] = _swingEnd;
        StartNextStep(now);
      } else {
        RegraspFailures += 1;
        Stage = "contact-wait";
      }
    }
    return baseTarget;
  }

  private V3 ManipulationTargets(double now, ContactReport contacts) {
    double elapsed = Math.Max(0.0, now - _manipulationStart);
    ResetToAnchors();
    V3 capStart = _anchors[Rig.FR];
    V3 baseTarget = MeanAnchors() + new V3(0.0, 0.0, 0.60);

    if (elapsed < 2.0) {
      _closures[Rig.FR] = 1.0 - MjMath.Smoothstep(elapsed / 2.0);
      Stage = "release-FR-for-service";
    } else if (elapsed < 4.0) {
      SetRailLatch(Rig.FR, false);
      double lift = MjMath.Smoothstep((elapsed - 2.0) / 2.0);
      _targets[Rig.FR] = capStart + new V3(0.0, 0.0, 0.30 * lift);
      _closures[Rig.FR] = 0.0;
      Stage = "retract-FR-above-rail";
    } else if (elapsed < 9.0) {
      SetRailLatch(Rig.FR, false);
      double move = MjMath.Smoothstep((elapsed - 4.0) / 5.0);
      V3 highStart = capStart + new V3(0.0, 0.0, 0.30);
      V3 highCap = Rig.CapTarget + new V3(0.0, 0.0, 0.30);
      _targets[Rig.FR] = (1.0 - move) * highStart + move * highCap;
      _closures[Rig.FR] = 0.0;
      Stage = "transfer-FR-above-cap";
    } else if (elapsed < 11.0) {
      SetRailLatch(Rig.FR, false);
      double descend = MjMath.Smoothstep((elapsed - 9.0) / 2.0);
      _targets[Rig.FR] = Rig.CapTarget + new V3(0.0, 0.0, 0.30 * (1.0 - descend));
      _closures[Rig.FR] = 0.0;
      Stage = "descend-FR-to-cap";
    } else if (elapsed < 13.5) {
      SetRailLatch(Rig.FR, false);
      _targets[Rig.FR] = Rig.CapTarget;
      _closures[Rig.FR] = 0.86 * MjMath.Smoothstep((elapsed - 11.0) / 2.5);
      Stage = "wrap-fingers-around-cap";
      if (contacts.CapFingerCount(Rig.FR) >= 2
          && _m.SiteError(_m.GraspSite[Rig.FR], _m.CapTargetSite) < 0.020) {
        CapGraspVerified = true;
      }
    } else {
      _targets[Rig.FR] = Rig.CapTarget;
      _closures[Rig.FR] = 0.86;
      if (!CapGraspVerified) {
        if (contacts.CapFingerCount(Rig.FR) >= 2
            && _m.SiteError(_m.GraspSite[Rig.FR], _m.CapTargetSite) < 0.020) {
          CapGraspVerified = true;
        }
        Stage = "cap-contact-wait";
      } else {
        double twist = MjMath.Smoothstep((elapsed - 13.5) / 10.0) * 0.95;
        _twists[Rig.FR] = twist;
        Stage = twist < 0.94 ? "physical-cap-twist" : "cap-task-complete";
      }
    }
    return baseTarget;
  }

  public void Update(double now, ContactReport contacts) {
    _m.ClearApplied();

    V3 baseTarget;
    if (!Autonomous || now < Rig.SettleTime) {
      ResetToAnchors();
      baseTarget = MeanTargets() + new V3(0.0, 0.0, 0.60);
      if (!Autonomous) Stage = "held";
    } else if (!_manipulating) {
      baseTarget = LocomotionTargets(now, contacts);
    } else if (now < _manipulationStart) {
      ResetToAnchors();
      baseTarget = MeanTargets() + new V3(0.0, 0.0, 0.60);
    } else {
      baseTarget = ManipulationTargets(now, contacts);
    }

    for (int leg = 0; leg < Rig.LegCount; leg++) {
      if (!_externalActive[leg]) continue;
      _targets[leg] = _externalTarget[leg];
      _closures[leg] = _externalClosure[leg];
      _twists[leg] = 0.0;
    }

    bool actualFrame = Autonomous && _manipulating && now >= _manipulationStart;
    Kinematics.ApplyTargets(_m, baseTarget, _targets, _closures, _twists, actualFrame, _limb, _fingers);

    // A limb under external control is driven by the Cartesian term alone.
    // Holding each joint actuator at its measured angle leaves it producing no
    // force of its own, so the limb has a single control path rather than the
    // inverse kinematics and the Cartesian term pulling against each other.
    // The finger actuators are left as commanded, since they carry the grip.
    for (int leg = 0; leg < Rig.LegCount; leg++) {
      if (_externalActive[leg]) _m.HoldLimb(leg);
    }
    _stabilization.WholeBody(baseTarget);
    _stabilization.EarthCentredDisplayOrbit();
    _stabilization.StationAttitudeHold();

    if (actualFrame) {
      for (int leg = 0; leg < Rig.LegCount; leg++) {
        if (leg != Rig.FR && !_externalActive[leg] && RailLatched(leg)) _m.HoldLimb(leg);
      }
    }

    for (int leg = 0; leg < Rig.LegCount; leg++) {
      bool capAttached = leg == Rig.FR && CapLatched;
      if (!RailLatched(leg) && !capAttached) {
        if (_externalActive[leg]) {
          _stabilization.CartesianJointControl(leg, _targets[leg], _externalForceLimit[leg],
                                               _externalStiffness[leg], _externalDamping[leg]);
        } else {
          double forceLimit = (leg == Rig.FR && CapGraspVerified) ? 35.0 : 120.0;
          _stabilization.CartesianJointControl(leg, _targets[leg], forceLimit);
        }
      }
    }
  }
}
}
