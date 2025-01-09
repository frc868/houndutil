package com.revrobotics.spark;

import com.revrobotics.REVLibError;
import com.revrobotics.jni.CANSparkJNI;

public class SparkBaseExtensions {
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
    public static REVLibError setInverted(SparkBase sparkBase, boolean isInverted) {
        return REVLibError.fromInt(
                CANSparkJNI.c_Spark_SetInverted(sparkBase.sparkHandle, isInverted));
    }

    /** Enable center aligned mode for the duty cycle sensor. */
    public static REVLibError enableCenterAlignedMode(SparkBase sparkMax) {
        return REVLibError.fromInt(
                CANSparkJNI.c_Spark_SetParameterBool(sparkMax.sparkHandle, 152, true));
    }

    /** Disable center aligned mode for the duty cycle sensor. */
    public static REVLibError disableCenterAlignedMode(SparkBase sparkMax) {
        return REVLibError.fromInt(
                CANSparkJNI.c_Spark_SetParameterBool(sparkMax.sparkHandle, 152, false));
    }

    /**
     * Enable mode which sets the output of the PID controllers to be voltage
     * instead of duty cycle.
     *
     * <p>
     * To disable, change or disable voltage compensation. Those settings will
     * overwrite this one
     */
    public static REVLibError enablePIDVoltageOutput(SparkBase sparkMax) {
        return REVLibError.fromInt(CANSparkJNI.c_Spark_SetParameterUint32(sparkMax.sparkHandle, 74, 1));
    }
}