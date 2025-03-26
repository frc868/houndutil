package com.techhounds.houndutil.houndlib.swerve;

import java.util.Random;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.techhounds.houndutil.houndlib.MotorHoldMode;
import com.techhounds.houndutil.houndlib.SparkConfigurator;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.math.MathUtil;
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
    private SparkMax driveMotor;
    private SparkMaxConfig driveConfig = new SparkMaxConfig();
    /** The motor used to control the wheel's steer angle. */
    @Log
    private SparkMax steerMotor;
    private SparkMaxConfig steerConfig = new SparkMaxConfig();

    /** The CANCoder used to tell the steer angle of the wheel. */
    @Log
    private CANcoder steerCanCoder;

    /** The simulated drive motor. */
    private DCMotorSim driveMotorSim;
    /** The simulated steer motor. */
    private DCMotorSim steerMotorSim;

    /** The constants required for this swerve module. */
    private final SwerveConstants SWERVE_CONSTANTS;

    /** The PID controller that corrects the drive motor's velocity. */
    @Log
    private final PIDController drivePidController;

    /** The PID controller that controls the steer motor's position. */
    @Log
    private final ProfiledPIDController steerPidController;

    /** The feedforward controller that controls the drive motor's velocity. */
    @Log
    private final SimpleMotorFeedforward driveFeedforward;

    private SwerveModuleState previousState = new SwerveModuleState();

    @Log(groups = "control")
    private double driveFeedbackVoltage = 0.0;
    @Log(groups = "control")
    private double driveFeedforwardVoltage = 0.0;
    @Log(groups = "control")
    private double steerFeedbackVoltage = 0.0;
    @Log
    private double simDriveEncoderVelocity = 0.0;
    @Log
    private boolean isUsingAbsoluteEncoder = true;
    @Log
    private boolean isDrivePidEnabled = true;

    @Log(name="drivetrain internal state")
    private SwerveModuleState internalState;

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
     *                              value to make it zero when the module is facing the
     *                              +x direction
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
        driveConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(SWERVE_CONSTANTS.DRIVE_CURRENT_LIMIT)
                .inverted(driveMotorInverted)
                .encoder
                    .positionConversionFactor(SWERVE_CONSTANTS.DRIVE_ENCODER_ROTATIONS_TO_METERS);
                    //.velocityConversionFactor(SWERVE_CONSTANTS.DRIVE_ENCODER_ROTATIONS_TO_METERS / 60.0);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        drivePidController = new PIDController(SWERVE_CONSTANTS.DRIVE_kP, SWERVE_CONSTANTS.DRIVE_kI,
                SWERVE_CONSTANTS.DRIVE_kD);
        driveFeedforward = new SimpleMotorFeedforward(SWERVE_CONSTANTS.DRIVE_kS, SWERVE_CONSTANTS.DRIVE_kV,
                SWERVE_CONSTANTS.DRIVE_kA);

        steerCanCoder = new CANcoder(canCoderChannel);
        MagnetSensorConfigs config = new MagnetSensorConfigs();
        config.SensorDirection = steerCanCoderInverted ? SensorDirectionValue.Clockwise_Positive
                : SensorDirectionValue.CounterClockwise_Positive;
        config.AbsoluteSensorDiscontinuityPoint = 0.5;
        config.MagnetOffset = steerCanCoderOffset;
        steerCanCoder.getConfigurator().apply(config);

        steerMotor = new SparkMax(steerMotorChannel, MotorType.kBrushless);
        steerConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(SWERVE_CONSTANTS.STEER_CURRENT_LIMIT)
            .inverted(steerMotorInverted)
            .encoder
                .positionConversionFactor(SWERVE_CONSTANTS.STEER_ENCODER_ROTATIONS_TO_RADIANS)
                .velocityConversionFactor(SWERVE_CONSTANTS.STEER_ENCODER_ROTATIONS_TO_RADIANS / 60.0);
        steerMotor.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        steerPidController = new ProfiledPIDController(
                SWERVE_CONSTANTS.STEER_kP, SWERVE_CONSTANTS.STEER_kI, SWERVE_CONSTANTS.STEER_kD,
                new TrapezoidProfile.Constraints(
                        SWERVE_CONSTANTS.MAX_STEER_VELOCITY_RADIANS_PER_SECOND,
                        SWERVE_CONSTANTS.MAX_STEER_ACCELERATION_RADIANS_PER_SECOND_SQUARED));

        steerMotor.getEncoder().setPosition(
                Units.rotationsToRadians(steerCanCoder.getAbsolutePosition().getValueAsDouble()));

        // TODO: add noise
        driveMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(SWERVE_CONSTANTS.DRIVE_GEARBOX_REPR,
                SWERVE_CONSTANTS.DRIVE_MOI, SWERVE_CONSTANTS.DRIVE_GEARING), SWERVE_CONSTANTS.DRIVE_GEARBOX_REPR);

        steerMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(SWERVE_CONSTANTS.STEER_GEARBOX_REPR,
                SWERVE_CONSTANTS.STEER_MOI, SWERVE_CONSTANTS.STEER_GEARING), SWERVE_CONSTANTS.STEER_GEARBOX_REPR);

        steerPidController.enableContinuousInput(0, 2 * Math.PI);
        steerPidController.setTolerance(0.05);

        if (RobotBase.isSimulation()) {
            // simulates different angles on startup
            steerCanCoder.getSimState().setRawPosition(randomizer.nextDouble() - 0.5);
            this.simDriveEncoderVelocity = 0.0;
        }

        internalState = new SwerveModuleState();
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

    /**
     * Exposes the drive motor for additional configuration.
     * 
     * @return the drive motor
     */
    public SparkMax getDriveMotor() {
        return driveMotor;
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
        driveConfig.idleMode(motorHoldMode == MotorHoldMode.BRAKE ? IdleMode.kBrake
        : IdleMode.kCoast);
        driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        
        steerConfig.idleMode(motorHoldMode == MotorHoldMode.BRAKE ? IdleMode.kBrake
        : IdleMode.kCoast);
        driveMotor.configure(steerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setDriveCurrentLimit(int currentLimit) {
        driveConfig.smartCurrentLimit(currentLimit);
    }

    @Override
    public void stop() {
        driveMotor.stopMotor();
        steerMotor.stopMotor();
    }

    /**
     * Internal common implementation for setting the state of the swerve module.
     * Updates simulation states of the module if needed.
     * 
     * @param state    the desired state of the swerve module
     * @param openLoop whether the drive motor should use open loop velocity control
     */
    private void setStateInternal(SwerveModuleState state, boolean openLoop) {
        internalState = state;
        isDrivePidEnabled = !openLoop;
        if (openLoop) {
            setVoltageDrive(state.speedMetersPerSecond
                    / SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND * 12.0);
        } else {
            drivePidController.reset();
            drivePidController.setSetpoint(state.speedMetersPerSecond);
        }

        steerPidController.setGoal(state.angle.getRadians());

        if (RobotBase.isSimulation()) {
            driveMotorSim.setInputVoltage(driveMotor.getAppliedOutput() > 12 ? 12 : driveMotor.getAppliedOutput());
            driveMotorSim.update(0.020);

            simDriveEncoderVelocity = driveMotorSim.getAngularVelocityRPM() / 60.0
                    * SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE;
            driveMotor.getEncoder().setPosition(driveMotorSim.getAngularPositionRotations()
                    * SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE);

            steerMotorSim.setInputVoltage(steerMotor.getAppliedOutput() > 12 ? 12 : steerMotor.getAppliedOutput());

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

    public void setVoltageSteer(double voltage) {
        if (RobotBase.isSimulation()){
            steerMotorSim.setInputVoltage(MathUtil.clamp(voltage, -12, 12));
            //System.out.println(voltage);
        }
        steerMotor.setVoltage(MathUtil.clamp(voltage, -12, 12));
    }

    public void setVoltageDrive(double voltage) {
        driveMotor.setVoltage(MathUtil.clamp(voltage, -12, 12));
    }

    public void steerPeriodic() {
        this.steerFeedbackVoltage = steerPidController.calculate(getWheelAngle().getRadians());
        setVoltageSteer(this.steerFeedbackVoltage);
        //steerMotor.getEncoder().setPosition(getWheelAngle().getRadians());
        if (RobotBase.isSimulation()){
            steerMotorSim.update(0.020);
        }
        if (steerPidController.atGoal()){
            System.out.println("at goal");
            System.out.println(steerPidController.getGoal().position);
        }
    }

    public void drivePeriodic() {
        if (isDrivePidEnabled) {
            this.driveFeedbackVoltage = drivePidController.calculate(getDriveMotorVelocity());
            this.driveFeedforwardVoltage = driveFeedforward.calculateWithVelocities(getDriveMotorVelocity(),
                    internalState.speedMetersPerSecond);
            setVoltageDrive(this.driveFeedbackVoltage + this.driveFeedforwardVoltage);
        }
    }
}