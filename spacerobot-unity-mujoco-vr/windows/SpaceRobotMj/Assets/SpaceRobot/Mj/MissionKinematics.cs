using System;

namespace SpaceRobot {

// Legs are addressed by index everywhere in the stepped path; the names are
// kept only for model lookups during setup.
public static class Rig {
  public const int FL = 0, FR = 1, RL = 2, RR = 3;
  public const int LegCount = 4;

  public static readonly string[] Legs = { "FL", "FR", "RL", "RR" };
  // Gait order from the reference implementation.
  public static readonly int[] StepOrder = { FL, RR, FR, RL };

  public static readonly V3[] Shoulder = {
    new V3( 0.36,  0.28, 0.0), new V3( 0.36, -0.28, 0.0),
    new V3(-0.36,  0.28, 0.0), new V3(-0.36, -0.28, 0.0),
  };
  public static readonly double[] Heading = { 0.0, 0.0, Math.PI, Math.PI };

  public static readonly V3[] InitialAnchors = {
    new V3(-2.25,  0.60, 0.58), new V3(-2.25, -0.60, 0.58),
    new V3(-3.15,  0.60, 0.58), new V3(-3.15, -0.60, 0.58),
  };

  public const double UpperLength = 0.46;
  public const double ForearmLength = 0.42;

  public const double OrbitRate = 0.002;
  public static readonly V3 EarthCentre = new V3(-10.0, 7.0, -5.0);

  public const double SettleTime = 3.0;
  public const double StepPeriod = 2.40;
  public const double Stride = 0.50;
  public const double SwingClearance = 0.22;
  public const int LocomotionSteps = 40;
  public static readonly V3 PalmOffset = new V3(0.0, 0.0, 0.090);
  public static readonly V3 CapTarget = new V3(3.05, -1.00, 0.86);

  public static readonly string[] LimbJoints =
    { "yaw", "pitch", "elbow", "wrist_yaw", "wrist_pitch", "wrist_roll" };
}

public static class Kinematics {

  // Writes yaw, pitch, elbow, wrist yaw/pitch/roll into limb[0..5].
  public static void SolveLimbIk(int leg, V3 relative, M3 desiredPalm, double[] limb) {
    double heading = Rig.Heading[leg];
    V3 local = M3.RotZ(-heading) * (relative - Rig.Shoulder[leg]);

    double yaw = Math.Atan2(local.y, local.x);
    double radial = Math.Sqrt(local.x * local.x + local.y * local.y);
    double distance2 = radial * radial + local.z * local.z;
    double cosElbow = MjMath.Clamp(
        (distance2 - Rig.UpperLength * Rig.UpperLength - Rig.ForearmLength * Rig.ForearmLength)
        / (2.0 * Rig.UpperLength * Rig.ForearmLength), -0.999, 0.999);
    double elbow = Math.Acos(cosElbow);
    double pitch = Math.Atan2(-local.z, radial)
                 - Math.Atan2(Rig.ForearmLength * Math.Sin(elbow),
                              Rig.UpperLength + Rig.ForearmLength * Math.Cos(elbow));

    M3 forearm = M3.RotZ(heading + yaw) * M3.RotY(pitch + elbow);
    MjMath.ZyxAngles(forearm.T * desiredPalm, out double wy, out double wp, out double wr);

    limb[0] = yaw; limb[1] = pitch; limb[2] = elbow;
    limb[3] = wy;  limb[4] = wp;    limb[5] = wr;
  }

  public static void InverseKinematics(
      int leg, V3 basePosition, V3 graspTarget, double palmTwist, double[] limb) {
    SolveLimbIk(leg, (graspTarget + Rig.PalmOffset) - basePosition, M3.RotZ(palmTwist), limb);
  }

  public static void ActualPoseInverseKinematics(
      MjModelAccess m, int leg, V3 graspTarget, double palmTwist, double[] limb) {
    M3 sr = m.StationMat, rr = m.RobotMat;
    V3 targetWorld = m.StationPos + sr * (graspTarget + Rig.PalmOffset);
    SolveLimbIk(leg, rr.T * (targetWorld - m.RobotPos), rr.T * sr * M3.RotZ(palmTwist), limb);
  }

  // Writes the eight finger joint targets: prox1..4 then dist1..4.
  public static void FingerTargets(double closure, double[] fingers) {
    closure = MjMath.Clamp(closure, 0.0, 1.0);
    for (int index = 0; index < 4; index++) {
      double side = index < 2 ? 1.0 : -1.0;
      fingers[index] = (1.0 - closure) * (side * 0.72) + closure * (-side * 0.36);
      fingers[4 + index] = closure * (-side * 0.68);
    }
  }

  public static void ApplyTargets(
      MjModelAccess m, V3 basePosition, V3[] graspTargets,
      double[] closures, double[] twists, bool actualFrame,
      double[] limbScratch, double[] fingerScratch) {
    for (int leg = 0; leg < Rig.LegCount; leg++) {
      if (actualFrame) {
        ActualPoseInverseKinematics(m, leg, graspTargets[leg], twists[leg], limbScratch);
      } else {
        InverseKinematics(leg, basePosition, graspTargets[leg], twists[leg], limbScratch);
      }
      FingerTargets(closures[leg], fingerScratch);
      m.CommandLimb(leg, limbScratch);
      m.CommandFingers(leg, fingerScratch);
    }
  }
}
}
