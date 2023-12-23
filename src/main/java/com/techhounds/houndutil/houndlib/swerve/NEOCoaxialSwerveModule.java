package com.techhounds.houndutil.houndlib.swerve;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

@LoggedObject
public class NEOCoaxialSwerveModule implements CoaxialSwerveModule {
    /** The motor used for driving. */
    @Log
    private CANSparkMax driveMotor;
    /** The motor used to control the wheel's steer angle. */
    @Log
    private CANSparkMax steerMotor;

    /** The encoder on the motor used for driving. */
    private RelativeEncoder driveEncoder;
    /** The encoder on the motor used to control the wheels. */
    private RelativeEncoder steerEncoder;
    /** The CANCoder used to tell the steer angle of the wheel. */
    @Log
    private CANcoder steerCanCoder;

    /** The PID controller that corrects the drive motor's velocity. */
    @Log
    private PIDController drivePidController;

    /** The PID controller that controls the steer motor's position. */
    @Log
    private PIDController steerPidController;

    /** The feedforward controller that controls the drive motor's velocity. */
    @Log
    private SimpleMotorFeedforward driveFeedforward;

    @Log(groups = "control")
    private double driveFeedbackVoltage = 0.0;
    @Log(groups = "control")
    private double driveFeedforwardVoltage = 0.0;
    @Log(groups = "control")
    private double turnFeedbackVoltage = 0.0;

    private double simDriveEncoderPosition;
    private double simDriveEncoderVelocity;
    private double simCurrentAngle;

    private SwerveConstants swerveConstants;

    /**
     * The offset of the CANCoder from the zero point, in radians. This will be
     * added to any measurements obtained from the CANCoder.
     */
    @Log
    private double turnCanCoderOffset;

    @Log
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
        driveMotor = SparkMaxConfigurator.create(
                driveMotorChannel, MotorType.kBrushless, driveMotorInverted,
                (s) -> s.setIdleMode(IdleMode.kBrake),
                (s) -> s.setSmartCurrentLimit(swerveConstants.DRIVE_CURRENT_LIMIT),
                (s) -> s.getEncoder().setPositionConversionFactor(swerveConstants.ENCODER_ROTATIONS_TO_METERS),
                (s) -> s.getEncoder().setVelocityConversionFactor(swerveConstants.ENCODER_ROTATIONS_TO_METERS / 60.0));

        driveEncoder = driveMotor.getEncoder();

        steerMotor = SparkMaxConfigurator.create(
                turnMotorChannel, MotorType.kBrushless, turnMotorInverted,
                (s) -> s.setIdleMode(IdleMode.kBrake),
                (s) -> s.setSmartCurrentLimit(swerveConstants.STEER_CURRENT_LIMIT),
                (s) -> s.getEncoder().setPositionConversionFactor(swerveConstants.STEER_ENCODER_ROTATIONS_TO_RADIANS),
                (s) -> s.getEncoder()
                        .setVelocityConversionFactor(swerveConstants.STEER_ENCODER_ROTATIONS_TO_RADIANS / 60.0));

        steerEncoder = steerMotor.getEncoder();

        steerCanCoder = new CANcoder(canCoderChannel);

        MagnetSensorConfigs config = new MagnetSensorConfigs();
        config.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        steerCanCoder.getConfigurator().apply(config);

        this.turnCanCoderOffset = turnCanCoderOffset;
        new Thread(() -> {
            try {
                Thread.sleep(3000);
                steerEncoder.setPosition(
                        Units.rotationsToRadians(steerCanCoder.getPosition().getValue()) + turnCanCoderOffset);
            } catch (Exception e) {
            }
        }).start();

        drivePidController = new PIDController(swerveConstants.DRIVE_kP, swerveConstants.DRIVE_kI,
                swerveConstants.DRIVE_kD);
        steerPidController = new PIDController(swerveConstants.STEER_kP, swerveConstants.STEER_kI,
                swerveConstants.STEER_kD);
        driveFeedforward = new SimpleMotorFeedforward(swerveConstants.DRIVE_kS, swerveConstants.DRIVE_kV,
                swerveConstants.DRIVE_kA);

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
    public Rotation2d getWheelAngle() {
        if (RobotBase.isReal())
            return new Rotation2d(
                    this.isUsingAbsoluteEncoder
                            ? Units.rotationsToRadians(steerCanCoder.getPosition().getValue()) + turnCanCoderOffset
                            : steerEncoder.getPosition());
        else
            return new Rotation2d(simCurrentAngle);
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

    @Override
    public void setState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getWheelAngle());

        driveMotor.setVoltage(state.speedMetersPerSecond
                / swerveConstants.MAX_DRIVING_VELOCITY_METERS_PER_SECOND * 12.0);

        if (RobotBase.isReal()) {
            this.turnFeedbackVoltage = steerPidController.calculate(getWheelAngle().getRadians(),
                    state.angle.getRadians());

            steerMotor.setVoltage(turnFeedbackVoltage);
        } else {
            simDriveEncoderVelocity = state.speedMetersPerSecond;
            simDriveEncoderPosition += state.speedMetersPerSecond * 0.020; // 20ms loop rate

            simCurrentAngle = state.angle.getRadians();

            steerCanCoder.getSimState().setRawPosition(simCurrentAngle);
        }
    }

    @Override
    public void setStateClosedLoop(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getWheelAngle());

        this.driveFeedbackVoltage = drivePidController.calculate(driveEncoder.getVelocity(),
                state.speedMetersPerSecond);
        this.driveFeedforwardVoltage = driveFeedforward.calculate(state.speedMetersPerSecond);

        driveMotor.setVoltage(driveFeedbackVoltage + driveFeedforwardVoltage);

        if (RobotBase.isReal()) {
            this.turnFeedbackVoltage = steerPidController.calculate(getWheelAngle().getRadians(),
                    state.angle.getRadians());

            steerMotor.setVoltage(turnFeedbackVoltage);
        } else {
            simDriveEncoderVelocity = state.speedMetersPerSecond;
            double distancePer20Ms = state.speedMetersPerSecond / 50.0;
            simDriveEncoderPosition += distancePer20Ms;

            simCurrentAngle = state.angle.getRadians();
            steerCanCoder.setPosition(simCurrentAngle);
        }

    }

}