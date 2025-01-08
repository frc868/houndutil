package com.techhounds.houndutil.houndlib.swerve;

import com.techhounds.houndutil.houndlib.MotorHoldMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Represents a coaxial swerve module (one motor for driving, one motor for
 * steering).
 */
public interface CoaxialSwerveModule {
    /**
     * Common constants for a coaxial swerve module. To initialize, create a
     * `static` block in your Constants class and set the values.
     */
    public static class SwerveConstants {
        /** The P gain for feedback velocity control on the drive wheel. */
        public double DRIVE_kP;
        /** The I gain for feedback velocity control on the drive wheel. */
        public double DRIVE_kI;
        /** The D gain for feedback velocity control on the drive wheel. */
        public double DRIVE_kD;
        /** The S gain for feedforward velocity control on the drive wheel. */
        public double DRIVE_kS;
        /** The V gain for feedforward velocity control on the drive wheel. */
        public double DRIVE_kV;
        /** The A gain for feedforward velocity control on the drive wheel. */
        public double DRIVE_kA;

        /** The P gain for feedback position control on the steer wheel. */
        public double STEER_kP;
        /** The I gain for feedback position control on the steer wheel. */
        public double STEER_kI;
        /** The D gain for feedback position control on the steer wheel. */
        public double STEER_kD;
        /** The S gain for feedforward position control on the steer wheel. */
        public double STEER_kS;
        /** The V gain for feedforward position control on the steer wheel. */
        public double STEER_kV;
        /** The A gain for feedforward position control on the steer wheel. */
        public double STEER_kA;

        /**
         * The number of rotations the drive motor makes for every one rotation of the
         * wheel.
         */
        public double DRIVE_GEARING;
        /**
         * The number of rotations the steer motor makes for every one azimuth rotation
         * of the wheel.
         */
        public double STEER_GEARING;
        /**
         * For a typical swerve module, the azimuth turn motor also drives the wheel a
         * nontrivial amount, which affects the accuracy of odometry and control. This
         * ratio
         * represents the number of rotations of the drive motor caused by a rotation of
         * the azimuth.
         */
        public double COUPLING_RATIO;
        /**
         * Multiplier to convert drive motor rotations to meters travelled by the wheel.
         * Typically WHEEL_CIRCUMFERENCE / DRIVE_GEARING.
         */
        public double DRIVE_ENCODER_ROTATIONS_TO_METERS;
        /**
         * Multiplier to convert steer motor rotations to the overall rotation of the
         * wheel.
         * Typically 2 * Math.PI / STEER_GEARING.
         */
        public double STEER_ENCODER_ROTATIONS_TO_RADIANS;
        /**
         * The circumference of the wheel, in meters. This is the radius of the wheel *
         * 2pi. This is best determined experimentally, as different wheel types
         * compress differently and produce different effective wheel radii. You should
         * update this value relatively often, as wheel wear will reduce the effective
         * wheel radius, which is used for positional control.
         */
        public double WHEEL_CIRCUMFERENCE;

        /**
         * The maximum velocity of the edge of the wheel in meters per second. This can
         * be determined theoretically based on the maximum RPM of the motor and the
         * gear ratio, but it is recommended to determine this empirically as the
         * effective maximum velocity will be slightly lower.
         * 
         * <p>
         * This is effectively the maximum velocity of the robot.
         */
        public double MAX_DRIVING_VELOCITY_METERS_PER_SECOND;
        /**
         * The maximum acceleration of the edge of the wheel in meters per second per
         * second. This should be determined empirically.
         * 
         * <p>
         * This is effectively the maximum acceleration of the robot.
         */
        public double MAX_DRIVING_ACCELERATION_METERS_PER_SECOND_SQUARED;
        /**
         * The maximum rotational velocity of the azimuth of the wheel. This can be
         * determined theoretically based on the maximum RPM of the motor and the
         * steering gear ratio, but it is recommended to determine this empirically as
         * the effective maximum velocity will be slightly lower.
         */
        public double MAX_STEER_VELOCITY_RADIANS_PER_SECOND;
        /**
         * The maximum rotational acceleration of the azimuth of the wheel. This can be
         * determined
         */
        public double MAX_STEER_ACCELERATION_RADIANS_PER_SECOND_SQUARED;

        /**
         * The stator current limit of the drive motor.
         * 
         * <p>
         * 
         * Since the torque output of a motor is directly proportional to stator
         * current, this should be set to the maximum value that will not allow the
         * wheels to slip. This number will be higher for higher-traction wheels. Do not
         * exceed the thermal specifications of the motor.
         */
        public int DRIVE_CURRENT_LIMIT;
        /**
         * The stator current limit of the steer motor.
         */
        public int STEER_CURRENT_LIMIT;

        /**
         * A {@link DCMotor} representation of the driving motor. On a typical swerve
         * module, call {@code getMotorName(1)} (i.e. {@code getKrakenX60(1)}).
         */
        public DCMotor DRIVE_GEARBOX_REPR;
        /**
         * A {@link DCMotor} representation of the steer motor. On a typical swerve
         * module, call {@code getMotorName(1)} (i.e. {@code getKrakenX60(1)}).
         */
        public DCMotor STEER_GEARBOX_REPR;

        /**
         * The moment of inertia of the drive motor in kg*m/s^2. Used for simulation. If
         * you are unsure, use 0.01.
         */
        public double DRIVE_MOI;
        /**
         * The moment of inertia of the steer motor in kg*m/s^2. Used for simulation. If
         * you are unsure, use 0.025.
         */
        public double STEER_MOI;
    }

    /**
     * Gets the position of the drive motor in meters, after any gear ratios.
     * 
     * @return the position of the drive motor
     */
    public double getDriveMotorPosition();

    /**
     * Gets the velocity of the drive motor in meters per second, after any gear
     * ratios.
     * 
     * @return the velocity of the drive motor in meters per second
     */
    public double getDriveMotorVelocity();

    /**
     * Gets the applied voltage output of the drive motor.
     * 
     * @return the applied voltage output of the drive motor
     */
    public double getDriveMotorVoltage();

    /**
     * Gets the rotational position of the azimuth of the wheel in radians, after
     * any gear ratios.
     * 
     * @return the position of the wheel
     */
    public double getSteerMotorPosition();

    /**
     * Gets the rotational velocity of the azimuth of the wheel in radians per
     * second, after any gear ratios.
     * 
     * @return the rotational velocity of the azimuth of the wheel in radians per
     *         second
     */
    public double getSteerMotorVelocity();

    /**
     * Gets the applied voltage output of the steer motor.
     * 
     * @return the applied voltage output of the steer motor
     */
    public double getSteerMotorVoltage();

    /**
     * Gets the angle of the wheel, where 0 is facing in the +x direction.
     * 
     * @return the angle of the wheel
     */
    public Rotation2d getWheelAngle();

    /**
     * Gets the position and angle of the swerve module as a
     * {@link SwerveModulePosition}.
     * 
     * @return the position of the swerve module
     */
    public SwerveModulePosition getPosition();

    /**
     * Gets the velocity and angle of the swerve module as a
     * {@link SwerveModuleState}.
     * 
     * @return the velocity and angle of the swerve module
     */
    public SwerveModuleState getState();

    /**
     * Sets both motors in the module to either brake or coast mode.
     * 
     * @param motorHoldMode the hold mode to set
     */
    public void setMotorHoldMode(MotorHoldMode motorHoldMode);

    /**
     * Sets the stator current limit of the drive motor, if you want to change it
     * mid-operation.
     * 
     * @param currentLimit the desired stator current limit
     */
    public void setDriveCurrentLimit(int currentLimit);

    /**
     * Commands the module to stop immediately.
     */
    public void stop();

    /**
     * Sets the state (velocity and angle) of the swerve module. The angle of the
     * module always used closed-loop positional control, but this uses open-loop
     * control on the drive motor (suitable for teleoperated driving).
     * 
     * @param state the desired {@link SwerveModuleState}
     */
    public void setState(SwerveModuleState state);

    /**
     * Sets the state (velocity and angle) of the swerve module. The angle of the
     * module always used closed-loop positional control, but this uses closed-loop
     * control on the drive motor (suitable for path following, or for automated
     * point-to-point movement).
     * 
     * @param state the desired {@link SwerveModuleState}
     */
    public void setStateClosedLoop(SwerveModuleState state);

}
