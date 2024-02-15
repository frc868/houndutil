package com.techhounds.houndutil.houndlib;

public class Utils {
    public static boolean limitMechanism(boolean isLowerTriggered, boolean isUpperTriggered, double value) {
        return (value > 0.0 && isUpperTriggered) || (value < 0.0 && isLowerTriggered);
    }

    public static double applySoftStops(double voltage, double position, double minPosition, double maxPosition) {
        return (voltage > 0.0 && position >= maxPosition) || (voltage < 0.0 && position <= minPosition) ? 0 : voltage;
    }
}
