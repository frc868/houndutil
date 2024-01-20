package com.techhounds.houndutil.houndlib;

public class Utils {
    public static boolean limitMechanism(boolean isLowerTriggered, boolean isUpperTriggered, double value) {
        return (value > 0.0 && isUpperTriggered) || (value < 0.0 && isLowerTriggered);
    }
}
