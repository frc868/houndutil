package com.techhounds.houndutil.houndlib.swerve;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
    // @Log
    // private PIDController drivePidController;

    // /** The PID controller that controls the steer motor's position. */
    // // @Log
    // // private ProfiledPIDController steerPidController;

    // /** The feedforward controller that controls the drive motor's velocity. */
    // @Log
    // private SimpleMotorFeedforward driveFeedforward;

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

    private final SwerveConstants SWERVE_CONSTANTS;

    private final VoltageOut driveVoltageRequest = new VoltageOut(0);
    private final VelocityVoltage driveVelocityRequest = new VelocityVoltage(0);
    private final MotionMagicVoltage steerPositionRequest = new MotionMagicVoltage(0);

    private SwerveModuleState previousState = new SwerveModuleState();

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
        driveConfig.Feedback.SensorToMechanismRatio = SWERVE_CONSTANTS.DRIVE_GEARING;
        driveConfig.CurrentLimits.SupplyCurrentLimit = SWERVE_CONSTANTS.DRIVE_CURRENT_LIMIT;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfig.CurrentLimits.SupplyCurrentThreshold = 300;
        driveConfig.CurrentLimits.SupplyTimeThreshold = 1;
        driveConfig.Slot0.kS = SWERVE_CONSTANTS.DRIVE_kS;
        driveConfig.Slot0.kV = SWERVE_CONSTANTS.DRIVE_kV;
        driveConfig.Slot0.kP = SWERVE_CONSTANTS.DRIVE_kP;
        driveConfig.Slot0.kI = SWERVE_CONSTANTS.DRIVE_kI;
        driveConfig.Slot0.kD = SWERVE_CONSTANTS.DRIVE_kD;
        driveConfig.MotionMagic.MotionMagicCruiseVelocity = SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND
                / SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE;
        driveConfig.MotionMagic.MotionMagicAcceleration = SWERVE_CONSTANTS.MAX_DRIVING_ACCELERATION_METERS_PER_SECOND_SQUARED
                / SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE;
        driveConfigurator.apply(driveConfig);

        steerCanCoder = new CANcoder(canCoderChannel);
        MagnetSensorConfigs config = new MagnetSensorConfigs();
        config.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        config.MagnetOffset = steerCanCoderOffset;
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
        steerConfig.CurrentLimits.SupplyCurrentLimit = SWERVE_CONSTANTS.STEER_CURRENT_LIMIT;
        steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        steerConfig.CurrentLimits.SupplyCurrentThreshold = 300;
        steerConfig.CurrentLimits.SupplyTimeThreshold = 1;

        steerConfig.Slot0.kP = SWERVE_CONSTANTS.STEER_kP;
        steerConfig.Slot0.kI = SWERVE_CONSTANTS.STEER_kI;
        steerConfig.Slot0.kD = SWERVE_CONSTANTS.STEER_kD;
        steerConfig.MotionMagic.MotionMagicCruiseVelocity = SWERVE_CONSTANTS.MAX_STEER_VELOCITY_RADIANS_PER_SECOND
                / (2 * Math.PI);
        steerConfig.MotionMagic.MotionMagicAcceleration = SWERVE_CONSTANTS.MAX_STEER_ACCELERATION_RADIANS_PER_SECOND_SQUARED
                / (2 * Math.PI);
        steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
        steerConfigurator.apply(steerConfig);

        // drivePidController = new PIDController(SWERVE_CONSTANTS.DRIVE_kP,
        // SWERVE_CONSTANTS.DRIVE_kI,
        // SWERVE_CONSTANTS.DRIVE_kD);

        // driveFeedforward = new SimpleMotorFeedforward(SWERVE_CONSTANTS.DRIVE_kS,
        // SWERVE_CONSTANTS.DRIVE_kV,
        // SWERVE_CONSTANTS.DRIVE_kA);

        driveMotorSim = new DCMotorSim(SWERVE_CONSTANTS.DRIVE_GEARBOX_REPR,
                SWERVE_CONSTANTS.DRIVE_GEARING,
                SWERVE_CONSTANTS.DRIVE_MOI);
        steerMotorSim = new DCMotorSim(SWERVE_CONSTANTS.STEER_GEARBOX_REPR,
                SWERVE_CONSTANTS.STEER_GEARING,
                SWERVE_CONSTANTS.STEER_MOI);

        // steerPidController.enableContinuousInput(0, 2 * Math.PI);

        // if (RobotBase.isSimulation()) {
        // this.simDriveEncoderPosition = 0.0;
        // this.simDriveEncoderVelocity = 0.0;
        // this.simCurrentAngle = 0.0;
        // }
    }

    @Override
    public double getDriveMotorPosition() {
        return driveMotor.getPosition().getValue() * SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE;
    }

    @Override
    public double getDriveMotorVelocity() {
        return driveMotor.getVelocity().getValue() * SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE;
    }

    @Override
    public double getDriveMotorVoltage() {
        return driveMotor.getMotorVoltage().getValue();
    }

    @Override
    public Rotation2d getWheelAngle() {
        return new Rotation2d(Units.rotationsToRadians(steerMotor.getPosition().getValue()));
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
        currentConfigs.SupplyCurrentLimit = currentLimit;
        currentConfigs.StatorCurrentLimitEnable = true;
        driveMotor.getConfigurator().apply(currentConfigs);
    }

    @Override
    public void stop() {
        driveMotor.setControl(driveVoltageRequest.withOutput(0));
        steerMotor.set(0);
    }

    private void setStateInternal(SwerveModuleState state, boolean openLoop) {
        if (openLoop) {
            driveMotor.setControl(driveVoltageRequest.withOutput(
                    state.speedMetersPerSecond / SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND * 12.0));
        } else {
            driveMotor.setControl(driveVelocityRequest
                    .withVelocity(state.speedMetersPerSecond / SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE)
                    .withAcceleration((state.speedMetersPerSecond - previousState.speedMetersPerSecond) / 0.020
                            / SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE));
            previousState = state;
        }

        steerMotor.setControl(steerPositionRequest.withPosition(state.angle.getRotations()));

        if (RobotBase.isSimulation()) {
            driveMotorSim.setInputVoltage(driveMotor.getSimState().getMotorVoltage());
            driveMotorSim.update(0.020);

            driveMotor.getSimState()
                    .setRotorVelocity(driveMotorSim.getAngularVelocityRPM() * SWERVE_CONSTANTS.DRIVE_GEARING / 60.0);
            driveMotor.getSimState()
                    .setRawRotorPosition(driveMotorSim.getAngularPositionRotations() * SWERVE_CONSTANTS.DRIVE_GEARING);

            steerMotorSim.setInputVoltage(steerMotor.getSimState().getMotorVoltage());
            steerMotorSim.update(0.020);

            steerMotor.getSimState().setRawRotorPosition(steerMotorSim.getAngularPositionRotations());
            steerCanCoder.getSimState().setRawPosition(steerMotorSim.getAngularPositionRotations());
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