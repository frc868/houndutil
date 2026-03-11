package com.techhounds.houndutil.houndlib;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * A collection of static utility functions used throughout the library, which
 * can also be used in user code.
 */
public class Utils {
    /**
     * Adjusts the command voltage based on the states of the upper and lower limit
     * switches to ensure the mechanism operates within its allowed range of motion.
     * 
     * If the upper limit switch is triggered and the voltage is positive
     * (indicating upward motion), the mechanism is stopped by returning 0. If the
     * lower limit switch is triggered and the voltage is negative
     * (indicating downward motion), the mechanism is stopped by returning 0. If
     * neither condition is met, the desired voltage is returned as-is.
     * 
     * @param isLowerTriggered true if the lower limit switch is triggered,
     *                         false otherwise.
     * @param isUpperTriggered true if the upper limit switch is triggered,
     *                         false otherwise.
     * @param voltage          the desired command voltage. Positive values
     *                         indicate upward motion, negative values indicate
     *                         downward motion.
     * @return the adjusted command voltage. If the motion
     *         is allowed, the input voltage is returned;
     *         otherwise, 0 is returned to stop the mechanism.
     */
    public static double applyMechanismLimits(double voltage, boolean isLowerTriggered, boolean isUpperTriggered) {
        if ((voltage > 0.0 && isUpperTriggered) || (voltage < 0.0 && isLowerTriggered)) {
            return 0.0;
        }
        return voltage;
    }

    /**
     * Enforces soft stops by suppressing commands that would drive the mechanism
     * beyond its allowed position range.
     *
     * The sign of the commanded motion determines the intended direction of motion.
     * If motion would move farther past a limit that has already been reached, a
     * zero voltage is returned. Otherwise, the commanded voltage is returned
     * unchanged.
     *
     * @param command     desired drive motion; positive values move toward
     *                    increasing position, negative values toward decreasing
     *                    position
     * @param position    current position
     * @param minPosition minimum allowed position
     * @param maxPosition maximum allowed position
     * @return the original command if within limits; otherwise zero
     */
    public static double applySoftStops(double command, double position, double minPosition, double maxPosition) {
        return (command > 0.0 && position >= maxPosition) || (command < 0.0 && position <= minPosition) ? 0 : command;
    }

    /**
     * Enforces angular soft stops by suppressing command voltages that would drive
     * the mechanism beyond its allowed position range.
     *
     * The sign of the commanded voltage determines the intended direction of
     * motion. If motion would move farther past a limit that has already been
     * reached, a zero voltage is returned. Otherwise, the commanded voltage is
     * returned unchanged.
     *
     * @param commandVoltage desired drive voltage; positive values move toward
     *                       increasing angle, negative values toward decreasing
     *                       angle
     * @param position       current angular position
     * @param minPosition    minimum allowed angular position
     * @param maxPosition    maximum allowed angular position
     * @return the original command voltage if within limits; otherwise zero
     */
    public static Voltage applySoftStops(Voltage commandVoltage, Angle position, Angle minPosition, Angle maxPosition) {
        return Volts.of(applySoftStops(commandVoltage.in(Volts), position.in(Radians), minPosition.in(Radians),
                maxPosition.in(Radians)));
    }

    /**
     * Enforces angular soft stops by suppressing command voltages that would drive
     * the mechanism beyond its allowed position range.
     *
     * The sign of the commanded voltage determines the intended direction of
     * motion. If motion would move farther past a limit that has already been
     * reached, a zero voltage is returned. Otherwise, the commanded voltage is
     * returned unchanged.
     *
     * @param commandVoltage desired drive voltage; positive values move toward
     *                       increasing angle, negative values toward decreasing
     *                       angle
     * @param position       current linear position
     * @param minPosition    minimum allowed linear position
     * @param maxPosition    maximum allowed linear position
     * @return the original command voltage if within limits; otherwise zero
     */
    public static Voltage applySoftStops(Voltage commandVoltage, Distance position, Distance minPosition,
            Distance maxPosition) {
        return Volts.of(applySoftStops(commandVoltage.in(Volts), position.in(Meters), minPosition.in(Meters),
                maxPosition.in(Meters)));
    }

    /**
     * Enforces soft stops by clamping commands that would drive the mechanism
     * beyond its allowed position range.
     *
     * If the commanded position is outside the allowed range, it will be clamped to
     * the nearest limit. Otherwise, the commanded position is returned unchanged.
     *
     * @param command     desired position
     * @param minPosition minimum allowed position
     * @param maxPosition maximum allowed position
     * @return the original command if within limits; otherwise max or min
     */
    public static double applySoftStops(double command, double minPosition, double maxPosition) {
        return MathUtil.clamp(command, minPosition, maxPosition);
    }

    /**
     * Enforces soft stops by clamping commands that would drive the mechanism
     * beyond its allowed position range.
     *
     * If the commanded position is outside the allowed range, it will be clamped to
     * the nearest limit. Otherwise, the commanded position is returned unchanged.
     *
     * @param command     desired angular position
     * @param minPosition minimum allowed position
     * @param maxPosition maximum allowed position
     * @return the original command if within limits; otherwise max or min
     */
    public static Angle applySoftStops(Angle commandPosition, Angle minPosition, Angle maxPosition) {
        return Radians
                .of(applySoftStops(commandPosition.in(Radians), minPosition.in(Radians), maxPosition.in(Radians)));
    }

    /**
     * Enforces soft stops by clamping commands that would drive the mechanism
     * beyond its allowed position range.
     *
     * If the commanded position is outside the allowed range, it will be clamped to
     * the nearest limit. Otherwise, the commanded position is returned unchanged.
     *
     * @param command     desired linear position
     * @param minPosition minimum allowed position
     * @param maxPosition maximum allowed position
     * @return the original command if within limits; otherwise max or min
     */
    public static Distance applySoftStops(Distance commandPosition, Distance minPosition,
            Distance maxPosition) {
        return Meters.of(applySoftStops(commandPosition.in(Meters), minPosition.in(Meters), maxPosition.in(Meters)));
    }

    /**
     * Converts a ChassisSpeeds to a Twist2d by extracting two dimensions (Y and Z).
     * chain
     *
     * @param speeds The original translation
     * @return The resulting translation
     */
    public static Twist2d toTwist2d(ChassisSpeeds speeds) {
        return new Twist2d(
                speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    }

    /**
     * Interpolates between two values.
     * 
     * @param start the start value
     * @param end   the end value
     * @param t     where between the two values to interpolate to (0 is start, 1 is
     *              end)
     * @return the interpolated value
     */
    public static double interpolate(double start, double end, double t) {
        return start + (end - start) * t;
    }

    /**
     * Interpolates between two values.
     * 
     * @param start the start value
     * @param end   the end value
     * @param t     where between the two values to interpolate to (0 is start, 1 is
     *              end)
     * @return the interpolated value
     */
    public static int interpolate(int start, int end, double t) {
        return (int) (start + (end - start) * t);
    }

    /**
     * Computes the interpolation fraction between two values.
     * 
     * @param start the start value
     * @param end   the end value
     * @param query the value to find the interpolation fraction for
     * @return the interpolation fraction
     */
    public static double inverseInterpolate(double start, double end, double query) {
        return (query - start) / (end - start);
    }

    public static double getLineDistance(Pose2d pose, Pose2d lineApexPose) {
        double cosTheta = lineApexPose.getRotation().getCos();
        double sinTheta = lineApexPose.getRotation().getSin();

        double A = -sinTheta;
        double B = cosTheta;
        double C = -(A * lineApexPose.getX() + B * lineApexPose.getY());

        // formula for distance between point and line given Ax + By + C = 0
        double distance = (A * pose.getX() + B * pose.getY() + C)
                / Math.sqrt(A * A + B * B);
        return distance;
    }

    public static Pose2d getClosestPoseOnLine(Pose2d pose, Pose2d lineApexPose) {
        double cosTheta = lineApexPose.getRotation().getCos();
        double sinTheta = lineApexPose.getRotation().getSin();

        double A = -sinTheta;
        double B = cosTheta;
        double C = -(A * lineApexPose.getX() + B * lineApexPose.getY());

        // Project the point onto the line using the formula for closest point
        // projection
        double xClosest = (B * (B * pose.getX() - A * pose.getY()) - A * C) / (A * A + B * B);
        double yClosest = (A * (-B * pose.getX() + A * pose.getY()) - B * C) / (A * A + B * B);

        return new Pose2d(xClosest, yClosest, lineApexPose.getRotation());
    }

    public static boolean shouldFlipValueToRed() {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
    }
}
