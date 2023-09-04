package com.techhounds.houndutil.houndlib.swerve;

import java.util.function.Supplier;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.techhounds.houndutil.houndlib.SparkMaxConfigurator;
import com.techhounds.houndutil.houndlog.interfaces.Log;
import com.techhounds.houndutil.houndlog.interfaces.LoggedObject;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;

@LoggedObject
public class NEOCoaxialSwerveModule {
    public static class SwerveConstants {
        public final double DRIVE_kP;
        public final double DRIVE_kI;
        public final double DRIVE_kD;
        public final double DRIVE_kS;
        public final double DRIVE_kV;
        public final double DRIVE_kA;

        public final double TURN_kP;
        public final double TURN_kI;
        public final double TURN_kD;

        public final double GEARING;
        public final double TURN_GEARING;
        public final double WHEEL_RADIUS_METERS;
        public final double MAX_DRIVING_VELOCITY_METERS_PER_SECOND;
        public final double MAX_DRIVING_ACCELERATION_METERS_PER_SECOND_SQUARED;
        public final double MAX_TURNING_VELOCITY_RADIANS_PER_SECOND;
        public final double MAX_TURNING_ACCELERATION_RADIANS_PER_SECOND_SQUARED;

        public final double ENCODER_DISTANCE_TO_METERS;
        public final double TURN_ENCODER_DISTANCE_TO_METERS;

        /**
         * Creates a new SwerveConstants object.
         */
        public SwerveConstants(
                double drivekP, double drivekI, double drivekD,
                double drivekS, double drivekV, double drivekA,
                double turnkP, double turnkI, double turnkD,
                double gearing, double turnGearing,
                double wheelRadiusMeters,
                double maxDrivingVelocityMetersPerSecond,
                double maxDrivingAccelerationMetersPerSecondSquared,
                double maxAngularVelocityRadiansPerSecond,
                double maxAngularAccelerationRadiansPerSecondSquared,
                double encoderDistanceToMeters, double turnEncoderDistanceToMeters) {
            this.DRIVE_kP = drivekP;
            this.DRIVE_kI = drivekI;
            this.DRIVE_kD = drivekD;
            this.DRIVE_kS = drivekS;
            this.DRIVE_kV = drivekV;
            this.DRIVE_kA = drivekA;
            this.TURN_kP = turnkP;
            this.TURN_kI = turnkI;
            this.TURN_kD = turnkD;

            this.GEARING = gearing;
            this.TURN_GEARING = turnGearing;
            this.WHEEL_RADIUS_METERS = wheelRadiusMeters;
            this.MAX_DRIVING_VELOCITY_METERS_PER_SECOND = maxDrivingVelocityMetersPerSecond;
            this.MAX_DRIVING_ACCELERATION_METERS_PER_SECOND_SQUARED = maxDrivingAccelerationMetersPerSecondSquared;
            this.MAX_TURNING_VELOCITY_RADIANS_PER_SECOND = maxAngularVelocityRadiansPerSecond;
            this.MAX_TURNING_ACCELERATION_RADIANS_PER_SECOND_SQUARED = maxAngularAccelerationRadiansPerSecondSquared;
            this.ENCODER_DISTANCE_TO_METERS = encoderDistanceToMeters;
            this.TURN_ENCODER_DISTANCE_TO_METERS = turnEncoderDistanceToMeters;
        }
    }

    /** The motor used for driving. */
    @Log(name = "Drive Motor")
    private CANSparkMax driveMotor;
    /** The motor used for turning. */
    @Log(name = "Turn Motor")
    private CANSparkMax turnMotor;

    /** The encoder on the motor used for driving. */
    private RelativeEncoder driveEncoder;
    /** The encoder on the motor used for turning. */
    private RelativeEncoder turnEncoder;
    /** The CANCoder used to tell the angle of the wheel. */
    @Log(name = "Turn CANCoder")
    private CANCoder turnCanCoder;

    /** The PID controller that corrects the drive motor's velocity. */
    @Log(name = "Drive PID Controller")
    private PIDController drivePIDController;

    /** The PID controller that controls the turning motor's position. */
    @Log(name = "Turn PID Controller")
    private PIDController turnPIDController;

    /** The feedforward controller that controls the drive motor's velocity. */
    private SimpleMotorFeedforward driveFeedforward;

    @Log(name = "Commanded Velocity", groups = "Control")
    private double commandedVelocity = 0.0;
    @Log(name = "Commanded Angle", groups = "Control")
    private double commandedAngle = 0.0;

    @Log(name = "Measured Velocity", groups = "Control")
    private Supplier<Double> measuredVelocity = () -> getState().speedMetersPerSecond;
    @Log(name = "Measured Angle", groups = "Control")
    private Supplier<Double> measuredAngle = () -> getState().angle.getRadians();

    @Log(name = "Drive PID Output", groups = "Control")
    private double drivePidOutput = 0.0;
    @Log(name = "Drive Feedforward Output", groups = "Control")
    private double driveFeedforwardOutput = 0.0;
    @Log(name = "Turn PID Output", groups = "Control")
    private double turnPIDOutput = 0.0;

    private double simDriveEncoderPosition;
    private double simDriveEncoderVelocity;
    private double simCurrentAngle;

    private SwerveConstants swerveConstants;

    /**
     * The offset of the CANCoder from the zero point, in radians. This will be
     * added to any measurements obtained from the CANCoder.
     */
    private double turnCanCoderOffset;

    /**
     * Whether or not the module is using the absolute encoder for positioning or
     * the relative encoder on the NEO. The NEOs encoder has a much higher sampling
     * rate, so it is recommended to use that.
     */
    @Log(name = "Is Using Absolute Encoder")
    private boolean isUsingAbsoluteEncoder;

    /**
     * Initalizes a SwerveModule.
     * 
     * @param name                 the name of the module (used for logging)
     * @param driveMotorChannel    the CAN ID of the drive motor
     * @param turnMotorChannel     the CAN ID of the turning motor
     * @param canCoderChannel      the CAN ID of the CANCoder
     * @param driveMotorInverted   if the drive motor is inverted
     * @param turnMotorInverted    if the turn motor is inverted
     * @param turnCanCoderInverted if the turn encoder is inverted
     * @param turnCanCoderOffset   the offset, in radians, to add to the CANCoder
     *                             value to make it zero when the module is straight
     */
    public NEOCoaxialSwerveModule(
            int driveMotorChannel,
            int turnMotorChannel,
            int canCoderChannel,
            boolean driveMotorInverted,
            boolean turnMotorInverted,
            boolean turnCanCoderInverted,
            double turnCanCoderOffset,
            SwerveConstants swerveConstants) {

        driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
        SparkMaxConfigurator.configure(driveMotor)
                .withIdleMode(IdleMode.kBrake)
                .withInverted(driveMotorInverted)
                .withCurrentLimit(40)
                .withPositionConversionFactor(swerveConstants.ENCODER_DISTANCE_TO_METERS, true)
                .burnFlash();

        driveEncoder = driveMotor.getEncoder();

        turnMotor = new CANSparkMax(turnMotorChannel, MotorType.kBrushless);
        SparkMaxConfigurator.configure(turnMotor)
                .withIdleMode(IdleMode.kBrake)
                .withInverted(turnMotorInverted)
                .withPositionConversionFactor(swerveConstants.TURN_ENCODER_DISTANCE_TO_METERS, true)
                .withCurrentLimit(15).burnFlash();

        turnEncoder = turnMotor.getEncoder();

        turnCanCoder = new CANCoder(canCoderChannel);

        // There is an issue with absolute position vs position in CANCoders, namely
        // that the abs pos is sent a lot less frequently than the normal pos (every
        // 100ms vs every 10ms). According to this post:
        // https://www.chiefdelphi.com/t/official-sds-mk3-mk4-code/397109/99, setting
        // the CANCoder to "Boot to Absolute" will fix this.
        turnCanCoder.setPositionToAbsolute();
        turnCanCoder.configSensorDirection(turnCanCoderInverted);
        turnCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        turnCanCoder.configFeedbackCoefficient(2 * Math.PI / 4096.0, "rad", SensorTimeBase.PerSecond); // radians/sec

        this.turnCanCoderOffset = turnCanCoderOffset;
        new Thread(() -> {
            try {
                Thread.sleep(3000);
                turnEncoder.setPosition(turnCanCoder.getPosition() + turnCanCoderOffset);
            } catch (Exception e) {
            }
        }).start();

        drivePIDController = new PIDController(swerveConstants.DRIVE_kP, swerveConstants.DRIVE_kI,
                swerveConstants.DRIVE_kD);
        turnPIDController = new PIDController(swerveConstants.TURN_kP, swerveConstants.TURN_kI,
                swerveConstants.TURN_kD);

        turnPIDController.enableContinuousInput(0, 2 * Math.PI);
        turnPIDController.enableContinuousInput(0, 2 * Math.PI);

        this.swerveConstants = swerveConstants;

        if (RobotBase.isSimulation()) {
            this.simDriveEncoderPosition = 0.0;
            this.simDriveEncoderVelocity = 0.0;
            this.simCurrentAngle = 0.0;
        }
    }

    /**
     * Gets the angle of the wheel, CCW+, 0 being facing straight forward.
     * 
     * @return the angle of the wheel.
     */
    @Log(name = "Wheel Angle")
    public Rotation2d getWheelAngle() {
        if (RobotBase.isReal())
            return new Rotation2d(
                    this.isUsingAbsoluteEncoder ? turnCanCoder.getPosition() + turnCanCoderOffset
                            : turnEncoder.getPosition());
        else
            return new Rotation2d(simCurrentAngle);
    }

    /**
     * Gets the position of the drive encoder.
     * 
     * @return the position of the drive encoder.
     */
    public double getDriveEncoderPosition() {
        if (RobotBase.isReal())
            return driveEncoder.getPosition();
        else
            return simDriveEncoderPosition;
    }

    /**
     * Gets the velocity of the drive encoder.
     * 
     * @return the velocity of the drive encoder.
     */
    public double getDriveEncoderVelocity() {
        if (RobotBase.isReal())
            return driveEncoder.getVelocity();
        else
            return simDriveEncoderVelocity;
    }

    /**
     * Gets the state of the swerve module.
     * 
     * @return the state of the swerve module
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveEncoderPosition(), getWheelAngle());
    }

    /**
     * Gets the state of the swerve module.
     * 
     * @return the state of the swerve module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveEncoderVelocity(), getWheelAngle());
    }

    /**
     * Sets the PID controller setpoints to the desired state.
     * 
     * @param state    the desired state of the swerve module
     * @param openLoop whether to run open loop controls
     * @param optimize whether to optimize the state of the modules
     */
    public void setState(SwerveModuleState state, boolean openLoop, boolean optimize) {
        if (optimize)
            state = SwerveModuleState.optimize(state, getWheelAngle());

        if (openLoop) {
            driveMotor.setVoltage(state.speedMetersPerSecond
                    / swerveConstants.MAX_DRIVING_VELOCITY_METERS_PER_SECOND * 12.0);

        } else {
            this.drivePidOutput = drivePIDController.calculate(driveEncoder.getVelocity(),
                    state.speedMetersPerSecond);
            this.driveFeedforwardOutput = driveFeedforward.calculate(state.speedMetersPerSecond);

            driveMotor.setVoltage(drivePidOutput + driveFeedforwardOutput);
            turnMotor.set(turnPIDOutput);
        }

        if (RobotBase.isReal()) {
            this.turnPIDOutput = turnPIDController.calculate(getWheelAngle().getRadians(),
                    state.angle.getRadians());

            turnMotor.setVoltage(turnPIDOutput);

        } else {
            simDriveEncoderVelocity = state.speedMetersPerSecond;
            double distancePer20Ms = state.speedMetersPerSecond / 50.0;
            simDriveEncoderPosition += distancePer20Ms;

            simCurrentAngle = state.angle.getRadians();
            turnCanCoder.setPosition(simCurrentAngle);
        }

    }

    /**
     * Sets the PID controller setpoints to the desired state.
     * Defaults to open loop controls and optimized state.
     * 
     * @param state the desired state of the swerve module
     */
    public void setState(SwerveModuleState state) {
        setState(state, true, true);
    }

    public void setUsingAbsoluteEncoder(boolean isUsingAbsoluteEncoder) {
        // if we are switching to using the integrated NEO motor, we have to reset it
        if (!isUsingAbsoluteEncoder && this.isUsingAbsoluteEncoder) {
            turnEncoder.setPosition(turnCanCoder.getPosition() + turnCanCoderOffset);
        }
        this.isUsingAbsoluteEncoder = isUsingAbsoluteEncoder;
    }

    /**
     * Reset the position of the encoder on the drive motor.
     */
    public void resetDriveEncoder() {
        driveEncoder.setPosition(0);
    }

    /**
     * Stops the swerve module.
     */
    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }

    public void setIdleMode(IdleMode idleMode) {
        driveMotor.setIdleMode(idleMode);
        turnMotor.setIdleMode(idleMode);
    }

    public void setDriveCurrentLimit(int currentLimit) {
        driveMotor.setSmartCurrentLimit(currentLimit);
    }

}