package com.techhounds.houndutil;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

/**
 * Global constants used throughout HoundUtil.
 * Note: this will need to be updated each year.
 */
public class HoundConstants {
    /** The loop time of the robot. Default 0.02s. */
    public static Time LOOP_TIME = Milliseconds.of(20);

    /** The length (x-axis) of the field */
    public static final Distance FIELD_LENGTH = Inches.of(651.22);
    /** The width (y-axis) of the field */
    public static final Distance FIELD_WIDTH = Inches.of(317.69);

    public static class Vision {
        /** Distance to reject trig pose calculations */
        public static Distance TRIG_POSE_REJECT_DIST = Meters.of(4);
        /** Height from the ground to reject pose estimates below the robot*/
        public static Distance MIN_POSE_REJECT_HEIGHT = Meters.of(-1);
        /** Height from the ground to reject pose estimates above the robot*/
        public static Distance MAX_POSE_REJECT_HEIGHT = Meters.of(1);

    }
}
