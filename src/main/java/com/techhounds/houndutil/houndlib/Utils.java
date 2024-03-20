package com.techhounds.houndutil.houndlib;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Utils {
    public static boolean limitMechanism(boolean isLowerTriggered, boolean isUpperTriggered, double value) {
        return (value > 0.0 && isUpperTriggered) || (value < 0.0 && isLowerTriggered);
    }

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
}
