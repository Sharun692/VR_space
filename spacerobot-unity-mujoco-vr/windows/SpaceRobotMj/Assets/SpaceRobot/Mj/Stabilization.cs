using System;
using Mujoco;

namespace SpaceRobot {

// Station keeping, attitude hold and limb Cartesian control. The jacobian
// buffer is allocated once and reused.
public unsafe class Stabilization {
  private static readonly double[] LimbTorqueLimits = { 35.0, 55.0, 45.0, 10.0, 10.0, 12.0 };

  private readonly MjModelAccess _m;
  private readonly double[] _jacobian;
  private readonly double[] _limb = new double[6];
  private readonly double[] _fingers = new double[8];

  public Stabilization(MjModelAccess m) {
    _m = m;
    _jacobian = new double[3 * m.Nv];
  }

  public static V3 DisplayOrbitVelocity(V3 reference) {
    V3 radial = reference - Rig.EarthCentre;
    return Rig.OrbitRate * new V3(-radial.y, radial.x, 0.0);
  }

  public void EarthCentredDisplayOrbit() {
    ApplyOrbit(_m.StationBody);
    ApplyOrbit(_m.RobotBody);
  }

  private void ApplyOrbit(int id) {
    V3 radial = _m.BodyPos(id) - Rig.EarthCentre;
    radial.z = 0.0;
    _m.AddXfrcForce(id, _m.Model->body_subtreemass[id] * (-(Rig.OrbitRate * Rig.OrbitRate) * radial));
  }

  public void StationAttitudeHold() {
    M3 r = _m.StationMat;
    V3 attitudeError = 0.5 * new V3(r.m21 - r.m12, r.m02 - r.m20, r.m10 - r.m01);
    V3 angularVelocity = _m.Qvel3(_m.StationDof + 3);
    V3 torque = -1200.0 * attitudeError - 300.0 * angularVelocity;
    double n = torque.Norm;
    if (n > 400.0) torque = torque * (400.0 / n);
    _m.AddXfrcTorque(_m.StationBody, torque);
  }

  public void WholeBody(V3 targetStationLocal) {
    M3 sr = _m.StationMat, rr = _m.RobotMat;
    V3 targetWorld = _m.StationPos + sr * targetStationLocal;

    V3 relativeLinear = _m.Qvel3(_m.RobotDof) - _m.Qvel3(_m.StationDof);
    V3 force = 85.0 * (targetWorld - _m.RobotPos) - 30.0 * relativeLinear;
    double fn = force.Norm;
    if (fn > 65.0) force = force * (65.0 / fn);

    M3 rel = sr.T * rr;
    V3 attitudeErrorStation = 0.5 * new V3(rel.m21 - rel.m12, rel.m02 - rel.m20, rel.m10 - rel.m01);
    V3 attitudeErrorWorld = sr * attitudeErrorStation;
    V3 relativeAngular = _m.Qvel3(_m.RobotDof + 3) - _m.Qvel3(_m.StationDof + 3);
    V3 torque = -180.0 * attitudeErrorWorld - 50.0 * relativeAngular;
    double tn = torque.Norm;
    if (tn > 90.0) torque = torque * (90.0 / tn);

    _m.AddXfrcForce(_m.RobotBody, force);
    _m.AddXfrcTorque(_m.RobotBody, torque);
    _m.AddXfrcForce(_m.StationBody, -1.0 * force);
    _m.AddXfrcTorque(_m.StationBody, -1.0 * torque);
  }

  // Stiffness and damping default to the values the autonomous mission uses.
  public const double DefaultStiffness = 420.0;
  public const double DefaultDamping = 28.0;

  public void CartesianJointControl(int leg, V3 targetStationLocal, double forceLimit,
                                    double stiffness = DefaultStiffness,
                                    double damping = DefaultDamping) {
    V3 targetWorld = _m.StationPos + _m.StationMat * targetStationLocal;
    int siteId = _m.GraspSite[leg];
    int nv = _m.Nv;

    fixed (double* jac = _jacobian) {
      MujocoLib.mj_jacSite(_m.Model, _m.Data, jac, null, siteId);

      V3 velocity = new V3(0, 0, 0);
      for (int d = 0; d < nv; d++) {
        double qv = _m.Data->qvel[d];
        velocity.x += jac[d] * qv;
        velocity.y += jac[nv + d] * qv;
        velocity.z += jac[2 * nv + d] * qv;
      }

      V3 taskForce = stiffness * (targetWorld - _m.SitePos(siteId)) - damping * velocity;
      double fn = taskForce.Norm;
      if (fn > forceLimit) taskForce = taskForce * (forceLimit / fn);

      for (int j = 0; j < 6; j++) {
        int dof = _m.LimbDof(leg, j);
        double torque = jac[dof] * taskForce.x
                      + jac[nv + dof] * taskForce.y
                      + jac[2 * nv + dof] * taskForce.z;
        _m.Data->qfrc_applied[dof] += MjMath.Clamp(torque, -LimbTorqueLimits[j], LimbTorqueLimits[j]);
      }
    }
  }

  // Reproduces initialize_model from the reference implementation.
  public void InitializeModel() {
    V3 initialOrbitVelocity = DisplayOrbitVelocity(new V3(0, 0, 0));
    _m.SetQvel3(_m.StationDof, initialOrbitVelocity);
    _m.SetQvel3(_m.RobotDof, initialOrbitVelocity);
    _m.SetQvel3(_m.StationDof + 3, new V3(0, 0, 0));
    _m.SetQvel3(_m.RobotDof + 3, new V3(0, 0, 0));

    V3 b = new V3(-2.70, 0.0, 1.18);
    for (int leg = 0; leg < Rig.LegCount; leg++) {
      Kinematics.InverseKinematics(leg, b, Rig.InitialAnchors[leg], 0.0, _limb);
      Kinematics.FingerTargets(1.0, _fingers);
      _m.SetLimbQpos(leg, _limb);
      _m.SetFingerQpos(leg, _fingers);
      _m.CommandLimb(leg, _limb);
      _m.CommandFingers(leg, _fingers);
      _m.SetEqActive(_m.RailLatchEq[leg], true);
    }
    _m.SetEqActive(_m.CapLatchEq, false);
    MujocoLib.mj_forward(_m.Model, _m.Data);

    var anchors = new V3[Rig.LegCount];
    var closures = new double[Rig.LegCount];
    var twists = new double[Rig.LegCount];
    for (int leg = 0; leg < Rig.LegCount; leg++) {
      anchors[leg] = Rig.InitialAnchors[leg]; closures[leg] = 1.0; twists[leg] = 0.0;
    }

    for (int i = 0; i < 1500; i++) {
      Kinematics.ApplyTargets(_m, b, anchors, closures, twists, false, _limb, _fingers);
      MujocoLib.mj_step(_m.Model, _m.Data);
    }
    _m.Data->time = 0.0;
  }
}
}
