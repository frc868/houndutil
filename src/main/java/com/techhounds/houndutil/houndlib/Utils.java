package com.techhounds.houndutil.houndlib;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

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
     * Adjusts the command voltage based on the current position of the mechanism
     * to enforce soft stops at the minimum and maximum allowed positions.
     * 
     * If the desired motion is upward (positive voltage) and the position
     * has reached or exceeded the maximum allowed position, the function
     * returns 0 to stop the motion. If the desired motion is downward (negative
     * voltage) and the position has reached or fallen below the minimum allowed
     * position, the function returns 0 to stop the motion. If neither condition is
     * met, the input voltage is returned as-is to allow the motion.
     * 
     * @param voltage     the desired command voltage. Positive values indicate
     *                    upward motion, negative values indicate downward motion.
     * @param position    the current position of the mechanism.
     * @param minPosition the minimum allowed position of the mechanism.
     * @param maxPosition the maximum allowed position of the mechanism.
     * @return the adjusted command voltage. If the motion is within
     *         the allowed range, the input voltage is returned;
     *         otherwise, 0 is returned to stop the mechanism.
     */
    public static double applySoftStops(double voltage, double position, double minPosition, double maxPosition) {
        return (voltage > 0.0 && position >= maxPosition) || (voltage < 0.0 && position <= minPosition) ? 0 : voltage;
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
}
