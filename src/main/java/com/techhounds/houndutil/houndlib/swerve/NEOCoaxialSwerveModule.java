package com.techhounds.houndutil.houndlib.swerve;

import java.util.Random;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.techhounds.houndutil.houndlib.MotorHoldMode;
import com.techhounds.houndutil.houndlib.SparkConfigurator;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * An implementation of a coaxial swerve module that uses a NEO or NEO 550 on
 * both the drive and steer motors (technically, any motor that uses the SPARK
 * MAX would work).
 * 
 * Supports simulated operation as well, with full closed-loop control.
 * 
 * You must call {@link SparkConfigurator#safeBurnFlash()} to ensure that all
 * motors are configured.
 */
@LoggedObject
public class NEOCoaxialSwerveModule implements CoaxialSwerveModule {
    /** The motor used for driving. */
    @Log
    private final SparkMax driveMotor;
    /** The motor used to control the wheel's steer angle. */
    @Log
    private final SparkMax steerMotor;

    /** The CANCoder used to tell the steer angle of the wheel. */
    @Log
    private final CANcoder steerCanCoder;

    /** The PID controller that corrects the drive motor's velocity. */
    @Log
    private final PIDController drivePidController;

    /** The PID controller that controls the steer motor's position. */
    @Log
    private final ProfiledPIDController steerPidController;

    /** The feedforward controller that controls the drive motor's velocity. */
    @Log
    private final SimpleMotorFeedforward driveFeedforward;

    @Log
    private final DCMotorSim driveMotorSim;
    @Log
    private final DCMotorSim steerMotorSim;

    @Log(groups = "control")
    private double driveFeedbackVoltage = 0.0;
    @Log(groups = "control")
    private double driveFeedforwardVoltage = 0.0;
    @Log(groups = "control")
    private double steerFeedbackVoltage = 0.0;

    @Log
    private double simDriveEncoderVelocity = 0.0;

    private final SwerveConstants SWERVE_CONSTANTS;

    @Log
    private boolean isUsingAbsoluteEncoder = false;

    @Log
    private double lastSpeed = 0;

    private Random randomizer = new Random();

    /**
     * Initalizes a SwerveModule.
     * 
     * @param name                  the name of the module (used for logging)
     * @param driveMotorChannel     the CAN ID of the drive motor
     * @param steerMotorChannel     the CAN ID of the steer motor
     * @param canCoderChannel       the CAN ID of the CANCoder
     * @param driveMotorInverted    if the drive motor is inverted
     * @param steerMotorInverted    if the steer motor is inverted
     * @param steerCanCoderInverted if the steer encoder is inverted
     * @param steerCanCoderOffset   the offset, in radians, to add to the CANCoder
     *                              value to make it zero when the module is
     *                              straight
     */
    public NEOCoaxialSwerveModule(
            int driveMotorChannel,
            int steerMotorChannel,
            int canCoderChannel,
            boolean driveMotorInverted,
            boolean steerMotorInverted,
            boolean steerCanCoderInverted,
            double steerCanCoderOffset,
            SwerveConstants swerveConstants) {
        this.SWERVE_CONSTANTS = swerveConstants;

        driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);
        steerMotor = new SparkMax(steerMotorChannel, MotorType.kBrushless);

        // TODO: use REV's new configurator
        // driveMotor = SparkConfigurator.createSparkMax(driveMotorChannel,
        // MotorType.kBrushless, driveMotorInverted,
        // (s) -> s.setIdleMode(IdleMode.kBrake),
        // (s) -> s.setSmartCurrentLimit(SWERVE_CONSTANTS.DRIVE_CURRENT_LIMIT),
        // (s) ->
        // s.getEncoder().setPositionConversionFactor(SWERVE_CONSTANTS.DRIVE_ENCODER_ROTATIONS_TO_METERS),
        // (s) -> s.getEncoder()
        // .setVelocityConversionFactor(SWERVE_CONSTANTS.DRIVE_ENCODER_ROTATIONS_TO_METERS
        // / 60.0));
        // steerMotor = SparkConfigurator.createSparkMax(steerMotorChannel,
        // MotorType.kBrushed, steerMotorInverted,
        // (s) -> s.setIdleMode(IdleMode.kBrake),
        // (s) -> s.setSmartCurrentLimit(SWERVE_CONSTANTS.STEER_CURRENT_LIMIT),
        // (s) ->
        // s.getEncoder().setPositionConversionFactor(SWERVE_CONSTANTS.STEER_ENCODER_ROTATIONS_TO_RADIANS),
        // (s) -> s.getEncoder()
        // .setVelocityConversionFactor(SWERVE_CONSTANTS.STEER_ENCODER_ROTATIONS_TO_RADIANS
        // / 60.0));

        steerCanCoder = new CANcoder(canCoderChannel);

        MagnetSensorConfigs config = new MagnetSensorConfigs();
        config.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config.AbsoluteSensorDiscontinuityPoint = 0.5;
        config.MagnetOffset = steerCanCoderOffset;
        steerCanCoder.getConfigurator().apply(config);

        new Thread(() -> {
            try {
                Thread.sleep(2000);
                steerMotor.getEncoder().setPosition(
                        Units.rotationsToRadians(steerCanCoder.getAbsolutePosition().getValueAsDouble()));
            } catch (Exception e) {
            }
        }).start();

        drivePidController = new PIDController(SWERVE_CONSTANTS.DRIVE_kP, SWERVE_CONSTANTS.DRIVE_kI,
                SWERVE_CONSTANTS.DRIVE_kD);
        steerPidController = new ProfiledPIDController(
                SWERVE_CONSTANTS.STEER_kP, SWERVE_CONSTANTS.STEER_kI, SWERVE_CONSTANTS.STEER_kD,
                new TrapezoidProfile.Constraints(
                        SWERVE_CONSTANTS.MAX_STEER_VELOCITY_RADIANS_PER_SECOND,
                        SWERVE_CONSTANTS.MAX_STEER_ACCELERATION_RADIANS_PER_SECOND_SQUARED));
        driveFeedforward = new SimpleMotorFeedforward(SWERVE_CONSTANTS.DRIVE_kS, SWERVE_CONSTANTS.DRIVE_kV,
                SWERVE_CONSTANTS.DRIVE_kA);

        // TODO: add noise
        driveMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(SWERVE_CONSTANTS.DRIVE_GEARBOX_REPR,
                SWERVE_CONSTANTS.DRIVE_MOI, SWERVE_CONSTANTS.DRIVE_GEARING), SWERVE_CONSTANTS.DRIVE_GEARBOX_REPR);

        steerMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(SWERVE_CONSTANTS.STEER_GEARBOX_REPR,
                SWERVE_CONSTANTS.STEER_MOI, SWERVE_CONSTANTS.STEER_GEARING), SWERVE_CONSTANTS.STEER_GEARBOX_REPR);

        steerPidController.enableContinuousInput(0, 2 * Math.PI);

        if (RobotBase.isSimulation()) {
            // simulates different angles on startup
            steerCanCoder.getSimState().setRawPosition(randomizer.nextDouble() - 0.5);
            this.simDriveEncoderVelocity = 0.0;
        }
    }

    @Override
    public double getDriveMotorPosition() {
        return driveMotor.getEncoder().getPosition();
    }

    @Override
    public double getDriveMotorVelocity() {
        if (RobotBase.isReal())
            return driveMotor.getEncoder().getVelocity();
        else
            return simDriveEncoderVelocity;
    }

    @Override
    public double getDriveMotorVoltage() {
        return driveMotor.getAppliedOutput();
    }

    @Override
    public double getSteerMotorPosition() {
        return steerMotor.getEncoder().getPosition();
    }

    @Override
    public double getSteerMotorVelocity() {
        return steerMotor.getEncoder().getVelocity();
    }

    @Override
    public double getSteerMotorVoltage() {
        return steerMotor.getAppliedOutput();
    }

    @Override
    public Rotation2d getWheelAngle() {
        return new Rotation2d(
                this.isUsingAbsoluteEncoder
                        ? Units.rotationsToRadians(steerCanCoder.getAbsolutePosition().getValueAsDouble())
                        : steerMotor.getEncoder().getPosition());
    }

    @Override
    @Log
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveMotorPosition(), getWheelAngle());
    }

    @Override
    @Log
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveMotorVelocity(), getWheelAngle());
    }

    @Override
    public void setMotorHoldMode(MotorHoldMode motorHoldMode) {
        // driveMotor.setIdleMode(motorHoldMode == MotorHoldMode.BRAKE ? IdleMode.kBrake
        // : IdleMode.kCoast);
        // steerMotor.setIdleMode(motorHoldMode == MotorHoldMode.BRAKE ? IdleMode.kBrake
        // : IdleMode.kCoast);
    }

    @Override
    public void setDriveCurrentLimit(int currentLimit) {
        // driveMotor.setSmartCurrentLimit(currentLimit);
    }

    @Override
    public void stop() {
        driveMotor.set(0);
        steerMotor.set(0);
    }

    /**
     * Internal common implementation for setting the state of the swerve module.
     * Updates simulation states of the module if needed.
     * 
     * @param state    the desired state of the swerve module
     * @param openLoop whether the drive motor should use open loop velocity control
     */
    private void setStateInternal(SwerveModuleState state, boolean openLoop) {
        if (openLoop) {
            driveMotor.setVoltage(state.speedMetersPerSecond
                    / SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND * 12.0);
        } else {
            this.driveFeedbackVoltage = drivePidController.calculate(getDriveMotorVelocity(),
                    state.speedMetersPerSecond);
            this.driveFeedforwardVoltage = driveFeedforward.calculateWithVelocities(getDriveMotorVelocity(),
                    state.speedMetersPerSecond);

            driveMotor.setVoltage(driveFeedbackVoltage + driveFeedforwardVoltage);
        }

        this.steerFeedbackVoltage = steerPidController.calculate(getWheelAngle().getRadians(),
                state.angle.getRadians());

        steerMotor.setVoltage(steerFeedbackVoltage);

        if (RobotBase.isSimulation()) {
            driveMotorSim.setInputVoltage(driveMotor.getAppliedOutput() > 12 ? 12 : driveMotor.getAppliedOutput());
            driveMotorSim.update(0.020);

            simDriveEncoderVelocity = driveMotorSim.getAngularVelocityRPM() / 60.0
                    * SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE;
            driveMotor.getEncoder().setPosition(driveMotorSim.getAngularPositionRotations()
                    * SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE);

            steerMotorSim.setInputVoltage(steerMotor.getAppliedOutput() > 12 ? 12 : steerMotor.getAppliedOutput());
            steerMotorSim.update(0.020);

            steerMotor.getEncoder().setPosition(steerMotorSim.getAngularPositionRad());
            steerCanCoder.getSimState().setRawPosition(Units.radiansToRotations(steerMotorSim.getAngularPositionRad()));
        }
    }

    @Override
    public void setState(SwerveModuleState state) {
        setStateInternal(state, true);
    }

    @Override
    public void setStateClosedLoop(SwerveModuleState state) {
        setStateInternal(state, false);
    }
}