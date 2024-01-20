package com.techhounds.houndutil.houndlib.swerve;

import java.util.Random;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.techhounds.houndutil.houndlib.MotorHoldMode;
import com.techhounds.houndutil.houndlog.interfaces.Log;
import com.techhounds.houndutil.houndlog.interfaces.LoggedObject;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

@LoggedObject
public class NEOCoaxialSwerveModule implements CoaxialSwerveModule {
    /** The motor used for driving. */
    @Log
    private CANSparkMax driveMotor;
    /** The motor used to control the wheel's steer angle. */
    @Log
    private CANSparkMax steerMotor;

    /** The CANCoder used to tell the steer angle of the wheel. */
    @Log
    private CANcoder steerCanCoder;

    /** The PID controller that corrects the drive motor's velocity. */
    @Log
    private PIDController drivePidController;

    /** The PID controller that controls the steer motor's position. */
    @Log
    private ProfiledPIDController steerPidController;

    /** The feedforward controller that controls the drive motor's velocity. */
    @Log
    private SimpleMotorFeedforward driveFeedforward;

    @Log
    private DCMotorSim driveMotorSim;
    @Log
    private DCMotorSim steerMotorSim;

    @Log(groups = "control")
    private double driveFeedbackVoltage = 0.0;
    @Log(groups = "control")
    private double driveFeedforwardVoltage = 0.0;
    @Log(groups = "control")
    private double turnFeedbackVoltage = 0.0;

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
     * @param name                 the name of the module (used for logging)
     * @param driveMotorChannel    the CAN ID of the drive motor
     * @param steerMotorChannel    the CAN ID of the turning motor
     * @param canCoderChannel      the CAN ID of the CANCoder
     * @param driveMotorInverted   if the drive motor is inverted
     * @param turnMotorInverted    if the turn motor is inverted
     * @param turnCanCoderInverted if the turn encoder is inverted
     * @param turnCanCoderOffset   the offset, in radians, to add to the CANCoder
     *                             value to make it zero when the module is straight
     */
    public NEOCoaxialSwerveModule(
            int driveMotorChannel,
            int steerMotorChannel,
            int canCoderChannel,
            boolean driveMotorInverted,
            boolean turnMotorInverted,
            boolean turnCanCoderInverted,
            double turnCanCoderOffset,
            SwerveConstants swerveConstants) {
        this.SWERVE_CONSTANTS = swerveConstants;

        driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
        driveMotor.restoreFactoryDefaults();
        driveMotor.setInverted(driveMotorInverted);
        driveMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.setSmartCurrentLimit(SWERVE_CONSTANTS.DRIVE_CURRENT_LIMIT);
        driveMotor.getEncoder().setPositionConversionFactor(SWERVE_CONSTANTS.DRIVE_ENCODER_ROTATIONS_TO_METERS);
        driveMotor.getEncoder()
                .setVelocityConversionFactor(SWERVE_CONSTANTS.DRIVE_ENCODER_ROTATIONS_TO_METERS / 60.0);
        driveMotor.burnFlash();

        steerMotor = new CANSparkMax(steerMotorChannel, MotorType.kBrushless);
        steerMotor.restoreFactoryDefaults();
        steerMotor.setInverted(turnMotorInverted);
        steerMotor.setIdleMode(IdleMode.kBrake);
        steerMotor.setSmartCurrentLimit(SWERVE_CONSTANTS.STEER_CURRENT_LIMIT);
        steerMotor.getEncoder().setPositionConversionFactor(SWERVE_CONSTANTS.STEER_ENCODER_ROTATIONS_TO_RADIANS);
        steerMotor.getEncoder().setVelocityConversionFactor(SWERVE_CONSTANTS.STEER_ENCODER_ROTATIONS_TO_RADIANS / 60.0);
        steerMotor.burnFlash();

        steerCanCoder = new CANcoder(canCoderChannel);

        MagnetSensorConfigs config = new MagnetSensorConfigs();
        config.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        config.MagnetOffset = turnCanCoderOffset;
        steerCanCoder.getConfigurator().apply(config);

        // this.turnCanCoderOffset = turnCanCoderOffset;

        new Thread(() -> {
            try {
                Thread.sleep(2000);
                steerMotor.getEncoder().setPosition(
                        Units.rotationsToRadians(steerCanCoder.getAbsolutePosition().getValue()));
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

        driveMotorSim = new DCMotorSim(SWERVE_CONSTANTS.DRIVE_GEARBOX_REPR, SWERVE_CONSTANTS.DRIVE_GEARING,
                SWERVE_CONSTANTS.DRIVE_MOI);
        steerMotorSim = new DCMotorSim(SWERVE_CONSTANTS.STEER_GEARBOX_REPR, SWERVE_CONSTANTS.STEER_GEARING,
                SWERVE_CONSTANTS.STEER_MOI);

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
    public Rotation2d getWheelAngle() {
        return new Rotation2d(
                this.isUsingAbsoluteEncoder
                        ? Units.rotationsToRadians(steerCanCoder.getAbsolutePosition().getValue())
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

    private void setStateInternal(SwerveModuleState state, boolean openLoop) {
        if (openLoop) {
            driveMotor.setVoltage(state.speedMetersPerSecond
                    / SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND * 12.0);
        } else {
            this.driveFeedbackVoltage = drivePidController.calculate(getDriveMotorVelocity(),
                    state.speedMetersPerSecond);
            this.driveFeedforwardVoltage = driveFeedforward.calculate(getDriveMotorVelocity(),
                    state.speedMetersPerSecond, 0.020);

            driveMotor.setVoltage(driveFeedbackVoltage + driveFeedforwardVoltage);
        }

        this.turnFeedbackVoltage = steerPidController.calculate(getWheelAngle().getRadians(),
                state.angle.getRadians());

        steerMotor.setVoltage(turnFeedbackVoltage);

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