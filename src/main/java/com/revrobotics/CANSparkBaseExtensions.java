package com.revrobotics;

import com.revrobotics.jni.CANSparkMaxJNI;

public class CANSparkBaseExtensions {
    /**
     * Checked function for setting controller inverted.
     *
     * <p>
     * This call has no effect if the controller is a follower. To invert a
     * follower, see the
     * follow() method.
     *
     * @param isInverted The state of inversion, true is inverted.
     */
    public static REVLibError setInverted(CANSparkBase sparkBase, boolean isInverted) {
        sparkBase.throwIfClosed();
        return REVLibError.fromInt(
                CANSparkMaxJNI.c_SparkMax_SetInverted(sparkBase.sparkMaxHandle, isInverted));
    }

    /** Enable center aligned mode for the duty cycle sensor. */
    public static REVLibError enableCenterAlignedMode(CANSparkBase sparkMax) {
        return REVLibError.fromInt(
                CANSparkMaxJNI.c_SparkMax_SetParameterBool(sparkMax.sparkMaxHandle, 152, true));
    }

    /** Disable center aligned mode for the duty cycle sensor. */
    public static REVLibError disableCenterAlignedMode(CANSparkBase sparkMax) {
        return REVLibError.fromInt(
                CANSparkMaxJNI.c_SparkMax_SetParameterBool(sparkMax.sparkMaxHandle, 152, false));
    }

    /**
     * Enable mode which sets the output of the PID controllers to be voltage
     * instead of duty cycle.
     *
     * <p>
     * To disable, change or disable voltage compensation. Those settings will
     * overwrite this one
     */
    public static REVLibError enablePIDVoltageOutput(CANSparkBase sparkMax) {
        return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetParameterUint32(sparkMax.sparkMaxHandle, 74, 1));
    }
}