using System;
using Mujoco;

namespace SpaceRobot {

// Per-step contact readings, in flat arrays so the stepped path allocates
// nothing. Finger sets are held as 4-bit masks (bit 0 is finger 1).
public class ContactReport {
  public readonly int[] RailFingerMask = new int[Rig.LegCount];
  public readonly int[] CapFingerMask = new int[Rig.LegCount];
  public readonly double[] GripForces = new double[Rig.LegCount];
  public int UnwantedContacts;
  public double MaximumPenetration;
  public double CapContactForce;

  public int RailFingerCount(int leg) => PopCount(RailFingerMask[leg]);
  public int CapFingerCount(int leg) => PopCount(CapFingerMask[leg]);

  // Four bits only; a loop is clearer than an intrinsic here.
  private static int PopCount(int mask) {
    int count = 0;
    while (mask != 0) { count += mask & 1; mask >>= 1; }
    return count;
  }

  public void Reset() {
    Array.Clear(RailFingerMask, 0, RailFingerMask.Length);
    Array.Clear(CapFingerMask, 0, CapFingerMask.Length);
    Array.Clear(GripForces, 0, GripForces.Length);
    UnwantedContacts = 0;
    MaximumPenetration = 0.0;
    CapContactForce = 0.0;
  }
}

// Classifies every geom once, so the per-step contact loop is integer work.
public unsafe class ContactClassifier {
  private static readonly string[] RobotTokens =
    { "robot_torso", "robot_top", "upper_geom", "forearm_geom", "palm_geom",
      "shoulder_shell", "elbow_shell" };

  private static readonly string[] StationPrefixes =
    { "rail_", "top_rung", "bottom_rung", "side_diag", "service_", "central_",
      "lab_", "node_", "panel_", "radiator_" };

  private readonly sbyte[] _fingerLeg;    // -1 when the geom is not a finger
  private readonly sbyte[] _fingerBit;    // 0..3, the finger's bit position
  private readonly bool[] _isRail;
  private readonly bool[] _isCapSurface;
  private readonly bool[] _isRobotPart;
  private readonly bool[] _isStationPart;

  private readonly ContactReport _report = new ContactReport();

  public ContactClassifier(MjModelAccess m) {
    int n = m.Ngeom;
    _fingerLeg = new sbyte[n]; _fingerBit = new sbyte[n];
    _isRail = new bool[n]; _isCapSurface = new bool[n];
    _isRobotPart = new bool[n]; _isStationPart = new bool[n];

    for (int i = 0; i < n; i++) {
      string name = m.GeomName(i);
      _fingerLeg[i] = -1;

      int marker = name.IndexOf("_finger", StringComparison.Ordinal);
      if (marker >= 0) {
        string legName = name.Substring(0, 2);
        for (int leg = 0; leg < Rig.LegCount; leg++) {
          if (Rig.Legs[leg] == legName) { _fingerLeg[i] = (sbyte)leg; break; }
        }
        string rest = name.Substring(marker + "_finger".Length);
        int underscore = rest.IndexOf('_');
        string digits = underscore >= 0 ? rest.Substring(0, underscore) : rest;
        _fingerBit[i] = (sbyte)(int.Parse(digits) - 1);
      }

      _isRail[i] = name.StartsWith("rail_y", StringComparison.Ordinal);
      _isCapSurface[i] = name.StartsWith("service_cap", StringComparison.Ordinal)
                      || name.StartsWith("cap_rib", StringComparison.Ordinal);
      foreach (var token in RobotTokens) if (name.Contains(token)) { _isRobotPart[i] = true; break; }
      foreach (var prefix in StationPrefixes)
        if (name.StartsWith(prefix, StringComparison.Ordinal)) { _isStationPart[i] = true; break; }
    }
  }

  public ContactReport Measure(MjModelAccess m) {
    _report.Reset();
    var force6 = stackalloc double[6];
    int ncon = m.Data->ncon;

    for (int i = 0; i < ncon; i++) {
      var contact = m.Data->contact[i];
      int g1 = contact.geom1, g2 = contact.geom2;

      MujocoLib.mj_contactForce(m.Model, m.Data, i, force6);
      double forceNorm = Math.Sqrt(
          force6[0] * force6[0] + force6[1] * force6[1] + force6[2] * force6[2]);
      if (-contact.dist > _report.MaximumPenetration) _report.MaximumPenetration = -contact.dist;

      int finger = _fingerLeg[g1] >= 0 ? g1 : (_fingerLeg[g2] >= 0 ? g2 : -1);
      if (finger >= 0) {
        int other = finger == g1 ? g2 : g1;
        int leg = _fingerLeg[finger];
        _report.GripForces[leg] += forceNorm;
        if (_isRail[other]) {
          _report.RailFingerMask[leg] |= 1 << _fingerBit[finger];
        } else if (_isCapSurface[other]) {
          _report.CapFingerMask[leg] |= 1 << _fingerBit[finger];
          _report.CapContactForce += forceNorm;
        } else {
          _report.UnwantedContacts++;
        }
      } else if ((_isRobotPart[g1] || _isRobotPart[g2])
              && (_isStationPart[g1] || _isStationPart[g2])
              && contact.dist < -0.001) {
        _report.UnwantedContacts++;
      }
    }
    return _report;
  }
}
}
