package com.techhounds.houndutil.houndlib.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.techhounds.houndutil.houndlib.MotorHoldMode;
import com.techhounds.houndutil.houndlog.SignalManager;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * An implementation of a coaxial swerve module that uses the Kraken X60 on both
 * the drive and steer motors.
 * 
 * Supports simulated operation as well, with full closed-loop control.
 * 
 * For status signals to update, you must call SignalManager.update().
 */
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

    /** The simulated drive motor. */
    private DCMotorSim driveMotorSim;
    /** The simulated steer motor. */
    private DCMotorSim steerMotorSim;

    /** The constants required for this swerve module. */
    private final SwerveConstants SWERVE_CONSTANTS;

    // CTRE's control modes required to command motors how we want
    private final VoltageOut driveVoltageRequest = new VoltageOut(0);
    private final VelocityVoltage driveVelocityRequest = new VelocityVoltage(0);
    private final MotionMagicVoltage steerPositionRequest = new MotionMagicVoltage(0);

    private SwerveModuleState previousState = new SwerveModuleState();

    // status signals to collect, stored here so we can update them all at once
    private final StatusSignal<Angle> drivePosition;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<AngularAcceleration> driveAcceleration;
    private final StatusSignal<Voltage> driveMotorVoltage;
    private final StatusSignal<Angle> steerPosition;
    private final StatusSignal<AngularVelocity> steerVelocity;
    private final StatusSignal<AngularAcceleration> steerAcceleration;
    private final StatusSignal<Voltage> steerMotorVoltage;

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
     *                              value to make it zero when the module facing the
     *                              +x direction
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
        if (RobotBase.isReal()) {
            driveConfig.CurrentLimits.StatorCurrentLimit = SWERVE_CONSTANTS.DRIVE_CURRENT_LIMIT;
            driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        }

        driveConfig.Slot0.kS = SWERVE_CONSTANTS.DRIVE_kS;
        driveConfig.Slot0.kV = SWERVE_CONSTANTS.DRIVE_kV;
        driveConfig.Slot0.kP = SWERVE_CONSTANTS.DRIVE_kP;
        driveConfig.Slot0.kI = SWERVE_CONSTANTS.DRIVE_kI;
        driveConfig.Slot0.kD = SWERVE_CONSTANTS.DRIVE_kD;
        driveConfigurator.apply(driveConfig);

        steerCanCoder = new CANcoder(canCoderChannel, canBus);
        MagnetSensorConfigs config = new MagnetSensorConfigs();
        config.SensorDirection = steerCanCoderInverted ? SensorDirectionValue.Clockwise_Positive
                : SensorDirectionValue.CounterClockwise_Positive;
        config.AbsoluteSensorDiscontinuityPoint = 0.5;
        config.MagnetOffset = steerCanCoderOffset;
        steerCanCoder.getConfigurator().apply(config);

        steerMotor = new TalonFX(steerMotorChannel, canBus);
        TalonFXConfigurator steerConfigurator = steerMotor.getConfigurator();
        TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        steerConfig.MotorOutput.Inverted = steerMotorInverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        steerConfig.Feedback.FeedbackRemoteSensorID = steerCanCoder.getDeviceID();
        steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        steerConfig.Feedback.SensorToMechanismRatio = 1.0;
        steerConfig.Feedback.RotorToSensorRatio = SWERVE_CONSTANTS.STEER_GEARING;
        if (RobotBase.isReal()) {
            steerConfig.CurrentLimits.StatorCurrentLimit = SWERVE_CONSTANTS.STEER_CURRENT_LIMIT;
            steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        }

        steerConfig.Slot0.kP = SWERVE_CONSTANTS.STEER_kP;
        steerConfig.Slot0.kI = SWERVE_CONSTANTS.STEER_kI;
        steerConfig.Slot0.kD = SWERVE_CONSTANTS.STEER_kD;
        steerConfig.Slot0.kS = SWERVE_CONSTANTS.STEER_kS;
        steerConfig.Slot0.kV = SWERVE_CONSTANTS.STEER_kV;
        steerConfig.Slot0.kA = SWERVE_CONSTANTS.STEER_kA;
        steerConfig.MotionMagic.MotionMagicCruiseVelocity = SWERVE_CONSTANTS.MAX_STEER_VELOCITY_RADIANS_PER_SECOND
                / (2 * Math.PI);
        steerConfig.MotionMagic.MotionMagicAcceleration = SWERVE_CONSTANTS.MAX_STEER_ACCELERATION_RADIANS_PER_SECOND_SQUARED
                / (2 * Math.PI);
        steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
        steerConfigurator.apply(steerConfig);

        driveMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(SWERVE_CONSTANTS.DRIVE_GEARBOX_REPR,
                SWERVE_CONSTANTS.DRIVE_MOI, SWERVE_CONSTANTS.DRIVE_GEARING), SWERVE_CONSTANTS.DRIVE_GEARBOX_REPR);

        steerMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(SWERVE_CONSTANTS.STEER_GEARBOX_REPR,
                SWERVE_CONSTANTS.STEER_MOI, SWERVE_CONSTANTS.STEER_GEARING), SWERVE_CONSTANTS.STEER_GEARBOX_REPR);

        drivePosition = driveMotor.getPosition();
        driveVelocity = driveMotor.getVelocity();
        driveAcceleration = driveMotor.getAcceleration();
        driveMotorVoltage = driveMotor.getMotorVoltage();
        steerPosition = steerMotor.getPosition();
        steerVelocity = steerMotor.getVelocity();
        steerAcceleration = steerMotor.getAcceleration();
        steerMotorVoltage = steerMotor.getMotorVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(250,
                drivePosition, driveVelocity, driveAcceleration, driveMotorVoltage,
                steerPosition, steerVelocity, steerAcceleration, steerMotorVoltage);

        // register all signals with the SignalManager so that any downstream callers
        // get updated signals
        SignalManager.register(
                drivePosition, driveVelocity, driveAcceleration, driveMotorVoltage,
                steerPosition, steerVelocity, steerAcceleration, steerMotorVoltage);
    }

    @Override
    public double getDriveMotorPosition() {
        return BaseStatusSignal.getLatencyCompensatedValue(drivePosition, driveVelocity).magnitude();
    }

    @Override
    public double getDriveMotorVelocity() {
        return BaseStatusSignal.getLatencyCompensatedValue(driveVelocity, driveAcceleration).magnitude();
    }

    @Override
    public double getDriveMotorVoltage() {
        return driveMotorVoltage.getValue().magnitude();
    }

    /**
     * Exposes the drive motor for additional configuration.
     * 
     * @return the drive motor
     */
    public TalonFX getDriveMotor() {
        return driveMotor;
    }

    @Override
    public double getSteerMotorPosition() {
        return BaseStatusSignal.getLatencyCompensatedValue(steerPosition, steerVelocity).magnitude();
    }

    @Override
    public double getSteerMotorVelocity() {
        return BaseStatusSignal.getLatencyCompensatedValue(steerVelocity, steerAcceleration).magnitude();
    }

    @Override
    public double getSteerMotorVoltage() {
        return steerMotorVoltage.getValue().magnitude();
    }

    /**
     * Exposes the steer motor for additional configuration.
     * 
     * @return the steer motor
     */
    public TalonFX getSteerMotor() {
        return steerMotor;
    }

    /**
     * Returns an array of the required signals for odometry.
     * 
     * @return the required signals for odometry
     */
    public BaseStatusSignal[] getSignals() {
        return new BaseStatusSignal[] { drivePosition, driveVelocity, steerPosition, steerVelocity };
    }

    @Override
    public Rotation2d getWheelAngle() {
        return Rotation2d.fromRotations(getSteerMotorPosition());
    }

    @Override
    @Log
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                // the position value of the drive motor is in rotations, so back out the
                // correct number of rotations due to coupling between the drive and steer
                // gears, then multiply back by circumference to get distance traveled in
                // meters
                (getDriveMotorPosition()
                        - getSteerMotorPosition() * SWERVE_CONSTANTS.COUPLING_RATIO
                                / SWERVE_CONSTANTS.DRIVE_GEARING)
                        * SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE,
                getWheelAngle());
    }

    @Override
    @Log
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveMotorVelocity() * SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE, getWheelAngle());
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

    /**
     * Internal common implementation for setting the state of the swerve module.
     * Updates simulation states of the module if needed.
     * 
     * @param state    the desired state of the swerve module
     * @param openLoop whether the drive motor should use open loop velocity control
     */
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