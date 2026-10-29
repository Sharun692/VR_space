using System;

namespace SpaceRobot {

// Double-precision helpers. MuJoCo works in doubles and the mission control
// law is sensitive to the difference, so Unity's float types are avoided here.
public struct V3 {
  public double x, y, z;
  public V3(double x, double y, double z) { this.x = x; this.y = y; this.z = z; }
  public static V3 operator +(V3 a, V3 b) => new V3(a.x + b.x, a.y + b.y, a.z + b.z);
  public static V3 operator -(V3 a, V3 b) => new V3(a.x - b.x, a.y - b.y, a.z - b.z);
  public static V3 operator *(double s, V3 a) => new V3(s * a.x, s * a.y, s * a.z);
  public static V3 operator *(V3 a, double s) => s * a;
  public double Norm => Math.Sqrt(x * x + y * y + z * z);
  public double this[int i] => i == 0 ? x : (i == 1 ? y : z);
  public override string ToString() => $"({x:R}, {y:R}, {z:R})";
}

// Row-major 3x3.
public struct M3 {
  public double m00, m01, m02, m10, m11, m12, m20, m21, m22;

  public static M3 Identity => new M3 { m00 = 1, m11 = 1, m22 = 1 };

  public static M3 FromRowMajor(double[] a, int o) => new M3 {
    m00 = a[o + 0], m01 = a[o + 1], m02 = a[o + 2],
    m10 = a[o + 3], m11 = a[o + 4], m12 = a[o + 5],
    m20 = a[o + 6], m21 = a[o + 7], m22 = a[o + 8]
  };

  public static M3 RotZ(double angle) {
    double c = Math.Cos(angle), s = Math.Sin(angle);
    return new M3 { m00 = c, m01 = -s, m02 = 0, m10 = s, m11 = c, m12 = 0, m20 = 0, m21 = 0, m22 = 1 };
  }

  public static M3 RotY(double angle) {
    double c = Math.Cos(angle), s = Math.Sin(angle);
    return new M3 { m00 = c, m01 = 0, m02 = s, m10 = 0, m11 = 1, m12 = 0, m20 = -s, m21 = 0, m22 = c };
  }

  public M3 T => new M3 {
    m00 = m00, m01 = m10, m02 = m20,
    m10 = m01, m11 = m11, m12 = m21,
    m20 = m02, m21 = m12, m22 = m22
  };

  public double Trace => m00 + m11 + m22;

  public static M3 operator *(M3 a, M3 b) => new M3 {
    m00 = a.m00 * b.m00 + a.m01 * b.m10 + a.m02 * b.m20,
    m01 = a.m00 * b.m01 + a.m01 * b.m11 + a.m02 * b.m21,
    m02 = a.m00 * b.m02 + a.m01 * b.m12 + a.m02 * b.m22,
    m10 = a.m10 * b.m00 + a.m11 * b.m10 + a.m12 * b.m20,
    m11 = a.m10 * b.m01 + a.m11 * b.m11 + a.m12 * b.m21,
    m12 = a.m10 * b.m02 + a.m11 * b.m12 + a.m12 * b.m22,
    m20 = a.m20 * b.m00 + a.m21 * b.m10 + a.m22 * b.m20,
    m21 = a.m20 * b.m01 + a.m21 * b.m11 + a.m22 * b.m21,
    m22 = a.m20 * b.m02 + a.m21 * b.m12 + a.m22 * b.m22
  };

  public static V3 operator *(M3 a, V3 v) => new V3(
    a.m00 * v.x + a.m01 * v.y + a.m02 * v.z,
    a.m10 * v.x + a.m11 * v.y + a.m12 * v.z,
    a.m20 * v.x + a.m21 * v.y + a.m22 * v.z);
}

public static class MjMath {
  public static double Clamp(double v, double lo, double hi) => v < lo ? lo : (v > hi ? hi : v);

  public static double Smoothstep(double x) {
    x = Clamp(x, 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
  }

  // Yaw, pitch, roll from a rotation matrix, matching the reference ZYX convention.
  public static void ZyxAngles(M3 m, out double yaw, out double pitch, out double roll) {
    pitch = Math.Asin(Clamp(-m.m20, -1.0, 1.0));
    if (Math.Abs(Math.Cos(pitch)) > 1e-7) {
      yaw = Math.Atan2(m.m10, m.m00);
      roll = Math.Atan2(m.m21, m.m22);
    } else {
      yaw = Math.Atan2(-m.m01, m.m11);
      roll = 0.0;
    }
  }
}
}
