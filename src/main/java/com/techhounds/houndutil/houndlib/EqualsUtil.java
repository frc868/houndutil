package com.techhounds.houndutil.houndlib;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

/**
 * Provides utility methods for comparing numerical values.
 * 
 * @see com.techhounds.houndutil.houndlib.swerve.SwerveSetpointGenerator
 */
public class EqualsUtil {
    /**
     * Returns whether two double values are equal within a given tolerance.
     *
     * @param a       the first value
     * @param b       the second value
     * @param epsilon the allowable difference between the values
     * @return true if {@code b} is within {@code epsilon} of {@code a}, otherwise
     *         false
     */
    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    /**
     * Returns whether two double values are equal within a given tolerance.
     *
     * @param a       the first value
     * @param b       the second value
     * @param epsilon the allowable difference between the values
     * @return true if {@code b} is within {@code epsilon} of {@code a}, otherwise
     *         false
     */
    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, 1e-9);
    }

    /**
     * Returns whether two double values are equal within a given tolerance.
     *
     * @param a       the first value
     * @param b       the second value
     * @param epsilon the allowable difference between the values
     * @return true if {@code b} is within {@code epsilon} of {@code a}, otherwise
     *         false
     */
    public static boolean epsilonEquals(Twist2d twist, Twist2d other) {
        return epsilonEquals(twist.dx, other.dx) && epsilonEquals(twist.dy, other.dy);
    }

    /**
     * Returns whether two double values are equal within a given tolerance.
     *
     * @param a       the first value
     * @param b       the second value
     * @param epsilon the allowable difference between the values
     * @return true if {@code b} is within {@code epsilon} of {@code a}, otherwise
     *         false
     */
    public static boolean epsilonEquals(Angle a, Angle b, Angle epsilon) {
        return epsilonEquals(a.in(Radians), b.in(Radians), epsilon.in(Radians));
    }

    /**
     * Returns whether two double values are equal within a given tolerance.
     *
     * @param a       the first value
     * @param b       the second value
     * @param epsilon the allowable difference between the values
     * @return true if {@code b} is within {@code epsilon} of {@code a}, otherwise
     *         false
     */
    public static boolean epsilonEquals(Distance a, Distance b, Distance epsilon) {
        return epsilonEquals(a.in(Meters), b.in(Meters, epsilon.in(Meters)));
    }

    /**
     * Returns whether two double values are equal within a given tolerance.
     *
     * @param a       the first value
     * @param b       the second value
     * @param epsilon the allowable difference between the values
     * @return true if {@code b} is within {@code epsilon} of {@code a}, otherwise
     *         false
     */
    public static boolean epsilonEquals(AngularVelocity a, AngularVelocity b, AngularVelocity epsilon) {
        return epsilonEquals(a.in(RadiansPerSecond), b.in(RadiansPerSecond), epsilon.in(RadiansPerSecond));
    }
}
