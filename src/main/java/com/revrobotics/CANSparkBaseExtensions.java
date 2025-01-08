package com.revrobotics;

import com.revrobotics.jni.CANSparkMaxJNI;

/**
 * An extension class designed to provide safe implementations of common SPARK
 * MAX parameter functions, and expose settings not accessible through REVLib.
 */
public class CANSparkBaseExtensions {
    /**
     * Common interface for inverting direction of a speed controller.
     * Returns a REVLibError instead of void.
     *
     * <p>
     * This call has no effect if the controller is a follower. To invert a
     * follower, see the follow() method.
     *
     * @param sparkBase  the controller object
     * @param isInverted The state of inversion, true is inverted.
     */
    public static REVLibError setInverted(CANSparkBase sparkBase, boolean isInverted) {
        sparkBase.throwIfClosed();
        return REVLibError.fromInt(
                CANSparkMaxJNI.c_SparkMax_SetInverted(sparkBase.sparkMaxHandle, isInverted));
    }

    /**
     * Enable center aligned mode for the absolute encoder. This makes the sensor
     * report [-0.5, 0.5] instead of [0, 1].
     * 
     * @param sparkBase the controller object
     */
    public static REVLibError enableCenterAlignedMode(CANSparkBase sparkMax) {
        return REVLibError.fromInt(
                CANSparkMaxJNI.c_SparkMax_SetParameterBool(sparkMax.sparkMaxHandle, 152, true));
    }

    /**
     * Disable center aligned mode for the duty cycle sensor. This makes the sensor
     * report [0, 1] instead of [-0.5, 0.5].
     * 
     * @param sparkBase the controller object
     */
    public static REVLibError disableCenterAlignedMode(CANSparkBase sparkMax) {
        return REVLibError.fromInt(
                CANSparkMaxJNI.c_SparkMax_SetParameterBool(sparkMax.sparkMaxHandle, 152, false));
    }

    /**
     * Enable mode which sets the output of the PID controllers to be voltage
     * instead of duty cycle. By default, SPARK motor controllers use duty cycle
     * output
     *
     * <p>
     * To disable, disable voltage compensation. Those settings will
     * overwrite this one.
     * 
     * @param sparkBase the controller object
     */
    public static REVLibError enablePIDVoltageOutput(CANSparkBase sparkMax) {
        return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetParameterUint32(sparkMax.sparkMaxHandle, 74, 1));
    }
}