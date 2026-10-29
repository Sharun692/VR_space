using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Mujoco;

namespace SpaceRobot {

// Named access to the MuJoCo model, with every index the stepped path needs
// resolved once at construction. Nothing here allocates per step.
public unsafe class MjModelAccess {
  public readonly MujocoLib.mjModel_* Model;
  public readonly MujocoLib.mjData_* Data;
  public readonly int Nv, Nq, Nu, Nbody, Ngeom;

  public readonly int StationBody, RobotBody;
  public readonly int StationDof, RobotDof;
  public readonly int CapHingeQpos;
  public readonly int[] RailLatchEq = new int[Rig.LegCount];
  public readonly int CapLatchEq;
  public readonly int[] RailTargetSite = new int[Rig.LegCount];
  public readonly int[] GraspSite = new int[Rig.LegCount];
  public readonly int CapTargetSite;

  // [leg][0..5] limb actuators and dof addresses, [leg][0..7] finger actuators.
  private readonly int[][] _limbActuator = new int[Rig.LegCount][];
  private readonly int[][] _limbQpos = new int[Rig.LegCount][];
  private readonly int[][] _limbDof = new int[Rig.LegCount][];
  private readonly int[][] _fingerActuator = new int[Rig.LegCount][];

  public MjModelAccess(MujocoLib.mjModel_* model, MujocoLib.mjData_* data) {
    Model = model; Data = data;
    Nv = (int)model->nv; Nq = (int)model->nq; Nu = (int)model->nu;
    Nbody = (int)model->nbody; Ngeom = (int)model->ngeom;

    StationBody = BodyId("station");
    RobotBody = BodyId("robot");
    StationDof = Model->jnt_dofadr[JointId("station_free")];
    RobotDof = Model->jnt_dofadr[JointId("robot_free")];
    CapHingeQpos = Model->jnt_qposadr[JointId("cap_hinge")];
    CapLatchEq = EqualityId("cap_latch_FR");
    CapTargetSite = SiteId("cap_target");

    for (int leg = 0; leg < Rig.LegCount; leg++) {
      string name = Rig.Legs[leg];
      RailLatchEq[leg] = EqualityId("rail_latch_" + name);
      RailTargetSite[leg] = SiteId("rail_target_" + name);
      GraspSite[leg] = SiteId(name + "_grasp_site");

      _limbActuator[leg] = new int[6];
      _limbQpos[leg] = new int[6];
      _limbDof[leg] = new int[6];
      for (int j = 0; j < 6; j++) {
        string joint = name + "_" + Rig.LimbJoints[j];
        _limbActuator[leg][j] = ActuatorId("act_" + joint);
        int jid = JointId(joint);
        _limbQpos[leg][j] = Model->jnt_qposadr[jid];
        _limbDof[leg][j] = Model->jnt_dofadr[jid];
      }

      _fingerActuator[leg] = new int[8];
      for (int f = 0; f < 4; f++) {
        _fingerActuator[leg][f] = ActuatorId($"act_{name}_finger{f + 1}_prox");
        _fingerActuator[leg][4 + f] = ActuatorId($"act_{name}_finger{f + 1}_dist");
      }
    }
  }

  public string GeomName(int id) {
    var ptr = MujocoLib.mj_id2name(Model, (int)MujocoLib.mjtObj.mjOBJ_GEOM, id);
    return ptr == IntPtr.Zero ? "" : Marshal.PtrToStringAnsi(ptr);
  }

  private int Lookup(MujocoLib.mjtObj type, string name) {
    int id = MujocoLib.mj_name2id(Model, (int)type, name);
    if (id < 0) throw new ArgumentException($"{type} '{name}' not found in the model.");
    return id;
  }

  public int JointId(string n) => Lookup(MujocoLib.mjtObj.mjOBJ_JOINT, n);
  public int ActuatorId(string n) => Lookup(MujocoLib.mjtObj.mjOBJ_ACTUATOR, n);
  public int SiteId(string n) => Lookup(MujocoLib.mjtObj.mjOBJ_SITE, n);
  public int BodyId(string n) => Lookup(MujocoLib.mjtObj.mjOBJ_BODY, n);
  public int EqualityId(string n) => Lookup(MujocoLib.mjtObj.mjOBJ_EQUALITY, n);

  public double CapAngle => Data->qpos[CapHingeQpos];

  public void CommandLimb(int leg, double[] values) {
    var actuators = _limbActuator[leg];
    for (int j = 0; j < 6; j++) Data->ctrl[actuators[j]] = values[j];
  }

  public void CommandFingers(int leg, double[] values) {
    var actuators = _fingerActuator[leg];
    for (int j = 0; j < 8; j++) Data->ctrl[actuators[j]] = values[j];
  }

  // Holds the limb at its measured pose, as the reference does for latched legs.
  public void HoldLimb(int leg) {
    var actuators = _limbActuator[leg];
    var qpos = _limbQpos[leg];
    for (int j = 0; j < 6; j++) Data->ctrl[actuators[j]] = Data->qpos[qpos[j]];
  }

  public void SetLimbQpos(int leg, double[] values) {
    var qpos = _limbQpos[leg];
    for (int j = 0; j < 6; j++) Data->qpos[qpos[j]] = values[j];
  }

  public void SetFingerQpos(int leg, double[] values) {
    // Finger joints follow their actuators one to one in this model.
    var actuators = _fingerActuator[leg];
    for (int j = 0; j < 8; j++) {
      int jointId = Model->actuator_trnid[actuators[j] * 2];
      Data->qpos[Model->jnt_qposadr[jointId]] = values[j];
    }
  }

  public int LimbDof(int leg, int joint) => _limbDof[leg][joint];

  public V3 BodyPos(int id) => new V3(Data->xpos[id * 3], Data->xpos[id * 3 + 1], Data->xpos[id * 3 + 2]);

  public M3 BodyMat(int id) {
    var p = Data->xmat + id * 9;
    return new M3 {
      m00 = p[0], m01 = p[1], m02 = p[2],
      m10 = p[3], m11 = p[4], m12 = p[5],
      m20 = p[6], m21 = p[7], m22 = p[8]
    };
  }

  public V3 StationPos => BodyPos(StationBody);
  public V3 RobotPos => BodyPos(RobotBody);
  public M3 StationMat => BodyMat(StationBody);
  public M3 RobotMat => BodyMat(RobotBody);

  public V3 SitePos(int id) =>
      new V3(Data->site_xpos[id * 3], Data->site_xpos[id * 3 + 1], Data->site_xpos[id * 3 + 2]);

  public void SetModelSitePos(int id, V3 v) {
    Model->site_pos[id * 3] = v.x;
    Model->site_pos[id * 3 + 1] = v.y;
    Model->site_pos[id * 3 + 2] = v.z;
  }

  public double SiteError(int a, int b) => (SitePos(a) - SitePos(b)).Norm;

  // The station moves, so limb targets are expressed in its frame.
  public V3 WorldToStationLocal(V3 world) => StationMat.T * (world - StationPos);
  public V3 StationLocalToWorld(V3 local) => StationPos + StationMat * local;

  public bool EqActive(int id) => Data->eq_active[id] != 0;
  public void SetEqActive(int id, bool on) { Data->eq_active[id] = (byte)(on ? 1 : 0); }

  public void AddXfrcForce(int bodyId, V3 f) {
    Data->xfrc_applied[bodyId * 6 + 0] += f.x;
    Data->xfrc_applied[bodyId * 6 + 1] += f.y;
    Data->xfrc_applied[bodyId * 6 + 2] += f.z;
  }

  public void AddXfrcTorque(int bodyId, V3 t) {
    Data->xfrc_applied[bodyId * 6 + 3] += t.x;
    Data->xfrc_applied[bodyId * 6 + 4] += t.y;
    Data->xfrc_applied[bodyId * 6 + 5] += t.z;
  }

  public void ClearApplied() {
    for (int i = 0; i < Nv; i++) Data->qfrc_applied[i] = 0.0;
    for (int i = 0; i < Nbody * 6; i++) Data->xfrc_applied[i] = 0.0;
  }

  public V3 Qvel3(int adr) => new V3(Data->qvel[adr], Data->qvel[adr + 1], Data->qvel[adr + 2]);
  public void SetQvel3(int adr, V3 v) {
    Data->qvel[adr] = v.x; Data->qvel[adr + 1] = v.y; Data->qvel[adr + 2] = v.z;
  }
}
}
