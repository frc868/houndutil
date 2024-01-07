package com.techhounds.houndutil.houndlib.swerve;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
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
public class KrakenCoaxialSwerveModule implements CoaxialSwerveModule {
    /** The motor used for driving. */
    @Log
    private TalonFX driveMotor;
    /** The motor used to control the wheel's steer angle. */
    @Log
    private TalonFX steerMotor;

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
    @Log
    private SwerveModulePosition lastPosition = getPosition();

    @Log(groups = "control")
    private double driveFeedbackVoltage = 0.0;
    @Log(groups = "control")
    private double driveFeedforwardVoltage = 0.0;
    @Log(groups = "control")
    private double turnFeedbackVoltage = 0.0;

    @Log
    private double simDriveEncoderPosition = 0.0;
    @Log
    private double simDriveEncoderVelocity = 0.0;
    @Log
    private double simCurrentAngle = 0.0;

    private final SwerveConstants SWERVE_CONSTANTS;

    /**
     * The offset of the CANCoder from the zero point, in radians. This will be
     * added to any measurements obtained from the CANCoder.
     */
    @Log
    private double turnCanCoderOffset;

    @Log
    private boolean isUsingAbsoluteEncoder;

    @Log
    private double accel = 0;

    /**
     * Initalizes a SwerveModule.
     *
     * @param name                  the name of the module (used for logging)
     * @param driveMotorChannel     the CAN ID of the drive motor
     * @param steerMotorChannel     the CAN ID of the turning motor
     * @param canCoderChannel       the CAN ID of the CANCoder
     * @param driveMotorInverted    if the drive motor is inverted
     * @param steerMotorInverted    if the turn motor is inverted
     * @param steerCanCoderInverted if the turn encoder is inverted
     * @param steerCanCoderOffset   the offset, in radians, to add to the CANCoder
     *                              value to make it zero when the module is
     *                              straight
     */
    public KrakenCoaxialSwerveModule(
            int driveMotorChannel,
            int steerMotorChannel,
            int canCoderChannel,
            String canBus,
            boolean driveMotorInverted,
            boolean steerMotorInverted,
            boolean steerCanCoderInverted,
            double steerCanCoderOffset,
            SwerveConstants swerveConstants) {
        this.SWERVE_CONSTANTS = swerveConstants;

        driveMotor = new TalonFX(driveMotorChannel, canBus);
        TalonFXConfigurator driveConfigurator = driveMotor.getConfigurator();
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.Inverted = driveMotorInverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.CurrentLimits.StatorCurrentLimit = SWERVE_CONSTANTS.DRIVE_CURRENT_LIMIT;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfigurator.apply(driveConfig);

        steerCanCoder = new CANcoder(canCoderChannel);
        MagnetSensorConfigs config = new MagnetSensorConfigs();
        config.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        steerCanCoder.getConfigurator().apply(config);

        steerMotor = new TalonFX(steerMotorChannel, canBus);
        TalonFXConfigurator steerConfigurator = steerMotor.getConfigurator();
        TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        steerConfig.MotorOutput.Inverted = driveMotorInverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        steerConfig.Feedback.FeedbackRemoteSensorID = steerCanCoder.getDeviceID();
        steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        steerConfig.Feedback.SensorToMechanismRatio = 1.0;
        steerConfig.Feedback.RotorToSensorRatio = SWERVE_CONSTANTS.STEER_GEARING;
        steerConfig.CurrentLimits.StatorCurrentLimit = SWERVE_CONSTANTS.STEER_CURRENT_LIMIT;
        steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        steerConfigurator.apply(steerConfig);

        drivePidController = new PIDController(SWERVE_CONSTANTS.DRIVE_kP,
                SWERVE_CONSTANTS.DRIVE_kI,
                SWERVE_CONSTANTS.DRIVE_kD);
        steerPidController = new ProfiledPIDController(
                SWERVE_CONSTANTS.STEER_kP, SWERVE_CONSTANTS.STEER_kI,
                SWERVE_CONSTANTS.STEER_kD,
                new TrapezoidProfile.Constraints(
                        SWERVE_CONSTANTS.MAX_STEER_VELOCITY_RADIANS_PER_SECOND,
                        SWERVE_CONSTANTS.MAX_STEER_ACCELERATION_RADIANS_PER_SECOND_SQUARED));
        driveFeedforward = new SimpleMotorFeedforward(SWERVE_CONSTANTS.DRIVE_kS,
                SWERVE_CONSTANTS.DRIVE_kV,
                SWERVE_CONSTANTS.DRIVE_kA);

        driveMotorSim = new DCMotorSim(SWERVE_CONSTANTS.DRIVE_GEARBOX_REPR,
                SWERVE_CONSTANTS.DRIVE_GEARING,
                SWERVE_CONSTANTS.DRIVE_MOI);
        steerMotorSim = new DCMotorSim(SWERVE_CONSTANTS.STEER_GEARBOX_REPR,
                SWERVE_CONSTANTS.STEER_GEARING,
                SWERVE_CONSTANTS.STEER_MOI);

        steerPidController.enableContinuousInput(0, 2 * Math.PI);

        if (RobotBase.isSimulation()) {
            this.simDriveEncoderPosition = 0.0;
            this.simDriveEncoderVelocity = 0.0;
            this.simCurrentAngle = 0.0;
        }
    }

    @Override
    public double getDriveMotorPosition() {
        if (RobotBase.isReal())
            return driveMotor.getPosition().getValue();
        else
            return simDriveEncoderPosition;
    }

    @Override
    public double getDriveMotorVelocity() {
        if (RobotBase.isReal())
            return driveMotor.getVelocity().getValue();
        else
            return simDriveEncoderVelocity;
    }

    @Override
    public Rotation2d getWheelAngle() {
        if (RobotBase.isReal())
            return new Rotation2d(
                    this.isUsingAbsoluteEncoder
                            ? Units.rotationsToRadians(steerCanCoder.getPosition().getValue()) +
                                    turnCanCoderOffset
                            : steerMotor.getPosition().getValue());
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
        MotorOutputConfigs driveConfigs = new MotorOutputConfigs();
        driveMotor.getConfigurator().refresh(driveConfigs);
        driveConfigs.NeutralMode = motorHoldMode == MotorHoldMode.BRAKE ? NeutralModeValue.Brake
                : NeutralModeValue.Coast;
        driveMotor.getConfigurator().apply(driveConfigs);

        MotorOutputConfigs steerConfigs = new MotorOutputConfigs();
        steerMotor.getConfigurator().refresh(steerConfigs);
        steerConfigs.NeutralMode = motorHoldMode == MotorHoldMode.BRAKE ? NeutralModeValue.Brake
                : NeutralModeValue.Coast;
        steerMotor.getConfigurator().apply(steerConfigs);
    }

    @Override
    public void setDriveCurrentLimit(int currentLimit) {
        CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
        driveMotor.getConfigurator().refresh(currentConfigs);
        currentConfigs.StatorCurrentLimit = currentLimit;
        currentConfigs.StatorCurrentLimitEnable = true;
        driveMotor.getConfigurator().apply(currentConfigs);
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
            this.driveFeedforwardVoltage = driveFeedforward.calculate(state.speedMetersPerSecond);

            driveMotor.setVoltage(driveFeedbackVoltage + driveFeedforwardVoltage);
        }

        this.turnFeedbackVoltage = steerPidController.calculate(getWheelAngle().getRadians(),
                state.angle.getRadians());

        steerMotor.setVoltage(turnFeedbackVoltage);

        if (RobotBase.isSimulation()) {
            driveMotorSim.setInputVoltage(
                    driveMotor.getMotorVoltage().getValue() > 12 ? 12 : driveMotor.getMotorVoltage().getValue());
            driveMotorSim.update(0.020);

            simDriveEncoderVelocity = driveMotorSim.getAngularVelocityRPM() / 60.0
                    * SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE;
            simDriveEncoderPosition = driveMotorSim.getAngularPositionRotations()
                    * SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE;

            steerMotorSim.setInputVoltage(
                    steerMotor.getMotorVoltage().getValue() > 12 ? 12 : steerMotor.getMotorVoltage().getValue());
            steerMotorSim.update(0.020);

            simCurrentAngle = steerMotorSim.getAngularPositionRad();

            steerCanCoder.getSimState().setRawPosition(simCurrentAngle);
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