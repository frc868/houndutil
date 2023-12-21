package com.techhounds.houndutil.houndlib.swerve;

import java.util.function.Supplier;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.techhounds.houndutil.houndlib.MotorHoldMode;
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
public class NEOCoaxialSwerveModule implements CoaxialSwerveModule {
    /** The motor used for driving. */
    @Log(name = "driveMotor")
    private CANSparkMax driveMotor;
    /** The motor used to control the wheel's steer angle. */
    @Log(name = "steerMotor")
    private CANSparkMax steerMotor;

    /** The encoder on the motor used for driving. */
    private RelativeEncoder driveEncoder;
    /** The encoder on the motor used to control the wheels. */
    private RelativeEncoder steerEncoder;
    /** The CANCoder used to tell the steer angle of the wheel. */
    @Log(name = "Steer CANCoder")
    private CANCoder steerCanCoder;

    /** The PID controller that corrects the drive motor's velocity. */
    @Log(name = "Drive PID Controller")
    private PIDController drivePIDController;

    /** The PID controller that controls the steer motor's position. */
    @Log(name = "Steer PID Controller")
    private PIDController steerPidController;

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
    private double turnPidOutput = 0.0;

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
                .withPositionConversionFactor(swerveConstants.ENCODER_ROTATIONS_TO_METERS, true)
                .burnFlash();

        driveEncoder = driveMotor.getEncoder();

        steerMotor = new CANSparkMax(turnMotorChannel, MotorType.kBrushless);
        SparkMaxConfigurator.configure(steerMotor)
                .withIdleMode(IdleMode.kBrake)
                .withInverted(turnMotorInverted)
                .withPositionConversionFactor(swerveConstants.STEER_ENCODER_ROTATIONS_TO_METERS, true)
                .withCurrentLimit(15).burnFlash();

        steerEncoder = steerMotor.getEncoder();

        steerCanCoder = new CANCoder(canCoderChannel);

        // There is an issue with absolute position vs position in CANCoders, namely
        // that the abs pos is sent a lot less frequently than the normal pos (every
        // 100ms vs every 10ms). According to this post:
        // https://www.chiefdelphi.com/t/official-sds-mk3-mk4-code/397109/99, setting
        // the CANCoder to "Boot to Absolute" will fix this.
        steerCanCoder.setPositionToAbsolute();
        steerCanCoder.configSensorDirection(turnCanCoderInverted);
        steerCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        steerCanCoder.configFeedbackCoefficient(2 * Math.PI / 4096.0, "rad", SensorTimeBase.PerSecond); // radians/sec

        this.turnCanCoderOffset = turnCanCoderOffset;
        new Thread(() -> {
            try {
                Thread.sleep(3000);
                steerEncoder.setPosition(steerCanCoder.getPosition() + turnCanCoderOffset);
            } catch (Exception e) {
            }
        }).start();

        drivePIDController = new PIDController(swerveConstants.DRIVE_kP, swerveConstants.DRIVE_kI,
                swerveConstants.DRIVE_kD);
        steerPidController = new PIDController(swerveConstants.STEER_kP, swerveConstants.STEER_kI,
                swerveConstants.STEER_kD);

        steerPidController.enableContinuousInput(0, 2 * Math.PI);
        steerPidController.enableContinuousInput(0, 2 * Math.PI);

        this.swerveConstants = swerveConstants;

        if (RobotBase.isSimulation()) {
            this.simDriveEncoderPosition = 0.0;
            this.simDriveEncoderVelocity = 0.0;
            this.simCurrentAngle = 0.0;
        }
    }

    @Override
    public double getDriveMotorPosition() {
        if (RobotBase.isReal())
            return driveEncoder.getPosition();
        else
            return simDriveEncoderPosition;
    }

    @Override
    public double getDriveMotorVelocity() {
        if (RobotBase.isReal())
            return driveEncoder.getVelocity();
        else
            return simDriveEncoderVelocity;
    }

    @Override
    @Log(name = "Wheel Angle")
    public Rotation2d getWheelAngle() {
        if (RobotBase.isReal())
            return new Rotation2d(
                    this.isUsingAbsoluteEncoder ? steerCanCoder.getPosition() + turnCanCoderOffset
                            : steerEncoder.getPosition());
        else
            return new Rotation2d(simCurrentAngle);
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveMotorPosition(), getWheelAngle());
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveMotorVelocity(), getWheelAngle());
    }

    @Override
    public void setMotorHoldMode(MotorHoldMode motorHoldMode) {
        driveMotor.setIdleMode(motorHoldMode == MotorHoldMode.BRAKE ? IdleMode.kBrake : IdleMode.kCoast);
        steerMotor.setIdleMode(motorHoldMode == MotorHoldMode.BRAKE ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void setDriveCurrentLimit(int currentLimit) {
        driveMotor.setSmartCurrentLimit(currentLimit);
    }

    @Override
    public void stop() {
        driveMotor.set(0);
        steerMotor.set(0);
    }

    @Override
    public void setState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getWheelAngle());

        driveMotor.setVoltage(state.speedMetersPerSecond
                / swerveConstants.MAX_DRIVING_VELOCITY_METERS_PER_SECOND * 12.0);

        if (RobotBase.isReal()) {
            this.turnPidOutput = steerPidController.calculate(getWheelAngle().getRadians(),
                    state.angle.getRadians());

            steerMotor.setVoltage(turnPidOutput);
        } else {
            simDriveEncoderVelocity = state.speedMetersPerSecond;
            simDriveEncoderPosition += state.speedMetersPerSecond * 0.020; // 20ms loop rate

            simCurrentAngle = state.angle.getRadians();
            steerCanCoder.setPosition(simCurrentAngle);
        }
    }

    @Override
    public void setStateClosedLoop(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getWheelAngle());

        this.drivePidOutput = drivePIDController.calculate(driveEncoder.getVelocity(),
                state.speedMetersPerSecond);
        this.driveFeedforwardOutput = driveFeedforward.calculate(state.speedMetersPerSecond);

        driveMotor.setVoltage(drivePidOutput + driveFeedforwardOutput);

        if (RobotBase.isReal()) {
            this.turnPidOutput = steerPidController.calculate(getWheelAngle().getRadians(),
                    state.angle.getRadians());

            steerMotor.setVoltage(turnPidOutput);
        } else {
            simDriveEncoderVelocity = state.speedMetersPerSecond;
            double distancePer20Ms = state.speedMetersPerSecond / 50.0;
            simDriveEncoderPosition += distancePer20Ms;

            simCurrentAngle = state.angle.getRadians();
            steerCanCoder.setPosition(simCurrentAngle);
        }

    }

}