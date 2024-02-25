package com.techhounds.houndutil.houndlib;

import edu.wpi.first.math.geometry.Twist2d;

public class EqualsUtil {
    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, 1e-9);
    }

    public static boolean epsilonEquals(Twist2d a, Twist2d b) {
        return epsilonEquals(a.dx, b.dx, 1e-9) && epsilonEquals(a.dy, b.dy, 1e-9)
                && epsilonEquals(a.dtheta, b.dtheta, 1e-9);
    }

    /** Extension methods for wpi geometry objects */
    public static class GeomExtensions {
        public static boolean epsilonEquals(Twist2d twist, Twist2d other) {
            return EqualsUtil.epsilonEquals(twist.dx, other.dx)
                    && EqualsUtil.epsilonEquals(twist.dy, other.dy);
        }
    }
}
