package com.techhounds.houndutil.houndlib.swerve;

import com.techhounds.houndutil.houndlib.MotorHoldMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;

public interface CoaxialSwerveModule {
    public static class SwerveConstants {
        public double DRIVE_kP;
        public double DRIVE_kI;
        public double DRIVE_kD;
        public double DRIVE_kS;
        public double DRIVE_kV;
        public double DRIVE_kA;

        public double STEER_kP;
        public double STEER_kI;
        public double STEER_kD;

        public double DRIVE_GEARING;
        public double STEER_GEARING;
        public double DRIVE_ENCODER_ROTATIONS_TO_METERS;
        public double STEER_ENCODER_ROTATIONS_TO_RADIANS;
        public double WHEEL_CIRCUMFERENCE;

        public double MAX_DRIVING_VELOCITY_METERS_PER_SECOND;
        public double MAX_DRIVING_ACCELERATION_METERS_PER_SECOND_SQUARED;
        public double MAX_STEER_VELOCITY_RADIANS_PER_SECOND;
        public double MAX_STEER_ACCELERATION_RADIANS_PER_SECOND_SQUARED;

        public int DRIVE_CURRENT_LIMIT;
        public int STEER_CURRENT_LIMIT;

        public DCMotor DRIVE_GEARBOX_REPR;
        public DCMotor STEER_GEARBOX_REPR;
        public double DRIVE_MOI;
        public double STEER_MOI;
    }

    public double getDriveMotorPosition();

    public double getDriveMotorVelocity();

    public double getDriveMotorVoltage();

    public Rotation2d getWheelAngle();

    public SwerveModulePosition getPosition();

    public SwerveModuleState getState();

    public void setMotorHoldMode(MotorHoldMode motorHoldMode);

    public void setDriveCurrentLimit(int currentLimit);

    public void stop();

    public void setState(SwerveModuleState state);

    public void setStateClosedLoop(SwerveModuleState state);

}
