package com.techhounds.houndutil.houndlib.subsystems.finished;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.techhounds.houndutil.houndlib.TuningConstants;
import com.techhounds.houndutil.houndlog.LogProfiles;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.loggers.LogGroup;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

/**
 * A flywheel mechanism. To use, after creating your subsystem file and class,
 * type
 * {@code extends FinishedFlywheel} after your class name (and before the
 * bracket).
 * <p>
 * All you need to do in order to make a Flywheel is call super() in the
 * flywheel's constructor, and the FinishedFlywheel will handle the logic.
 * <p>
 * If you want to add custom commands, make sure to utilize the commands built
 * into the intake.
 * <p>
 * <h3>Built in commands:</h3>
 * <p>
 * {@code stopCommand()} stops the system
 * <p>
 * {@code spinAtVelocityCommand(Supplier<AngularVelocity>)} runs the flywheel at
 * a given angular velocity
 * <p>
 * {@code setOverridenSpeedCommand(Supplier<Double>)} runs the flywheel at a
 * given speed [-1,1]
 * <p>
 * {@code coastMotorsCommand()} sets the motors to Coast until interrupted
 * 
 * 
 */
public abstract class FinishedFlywheel extends SubsystemBase {

    private final TalonConstants[] TALON_CONSTANTS;
    private final boolean ARE_FOLLOWERS;
    private final String NAME;
    private final Current CURRENT_LIMIT;
    private final double GEAR_RATIO;
    private final NeutralModeValue NEUTRAL;
    private final CANBus CANBUS;

    private final DCMotor SIM_MOTOR_PLANT;
    private final MomentOfInertia MOI;

    private final TuningConstants K;

    public AngularVelocity goalVelocity = RotationsPerSecond.zero();
    private final TalonFXConfiguration config = new TalonFXConfiguration();
    private final TalonFX[] motors;
    private final FlywheelSim[] sim;
    private final StrictFollower followerRequest;
    private final VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(true).withUseTimesync(true);
    private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0);

    /**
     * Gets the velocity of the flywheel. 0 should indicate it being stopped, and
     * the velocity should increase in the forward direction (i.e. the velocity
     * should be positive in the "correct" direction).
     * 
     * @return the velocity of the flywheel
     */
    public AngularVelocity getVelocity() {
        double total = 0.0;
        int i = 0;
        for (TalonFX motor : motors) {
            if (motor != null) {
                total = total + motor.getVelocity().getValue().in(RotationsPerSecond);
                i++;
            }
        }
        if (i == 0) {
            return RotationsPerSecond.zero();
        }

        total /= i;

        return RotationsPerSecond.of(total);
    }

    /**
     * Explicit function to set the voltage of the motors attached to the elevator,
     * should handle safeties and clamping here.
     * 
     * @param voltage the voltage to apply to the motors, [-12, 12]
     */
    public void setVoltage(Voltage voltage) {
        setMotorsControl(voltageRequest.withOutput(MathUtil.clamp(voltage.in(Volts), -12, 12)));
    }

    /**
     * Creates a command that continuously spins the flywheel at a specific velocity
     * until cancelled. Note that this is *not* intended to self-cancel after
     * reaching its setpoint and defer to a default command.
     * 
     * @param goalVelocitySupplier a supplier of a velocity to spin at
     * @return the command
     */
    public Command spinAtVelocityCommand(Supplier<AngularVelocity> goalVelocitySupplier) {
        return runEnd(() -> {
            goalVelocity = goalVelocitySupplier.get();
            setMotorsControl(velocityRequest.withVelocity(goalVelocity.in(RotationsPerSecond)));
        }, () -> stop()).withName(NAME + ".spinAtVelocity");
    }

    /**
     * Creates a command that manually sets the speed of the mechanism. Useful for
     * overriding PID control.
     * 
     * @apiNote use {@code setVoltage()} internally in order to maintain safeties
     *          and clamping
     * 
     * @param speed the speed [-1,1]
     * @return
     */
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        return runEnd(
                () -> setVoltage(Volts.of(speed.get() * 12)),
                () -> stop()).withName(NAME + ".setOverridenSpeedCommand");
    }

    /**
     * Creates a command stops the motor and sets it to coast mode, to allow for
     * moving the mechanism manually.
     * 
     * @apiNote use
     *          {@code .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)}
     *          for safety
     * @return the command
     */
    public Command coastMotorsCommand() {
        return startEnd(
                () -> {
                    stop();
                    for (TalonFX motor : motors) {
                        motor.setNeutralMode(NeutralModeValue.Coast);
                    }
                },
                () -> {
                    for (TalonFX motor : motors) {
                        motor.setNeutralMode(NEUTRAL);
                    }
                    goalVelocity = getVelocity();
                }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming).withName(NAME + ".coastMotors");
    }

    @Override
    public void simulationPeriodic() {
        if (ARE_FOLLOWERS) {
            sim[0].setInputVoltage(motors[0].getMotorVoltage().getValueAsDouble());
            sim[0].update(0.020);

            for (int i = 0; i < motors.length; i++) {
                int a = TALON_CONSTANTS[i].INVERT == InvertedValue.CounterClockwise_Positive ? 1 : -1;

                motors[i].getSimState().setRotorVelocity(sim[0].getAngularVelocity().times(GEAR_RATIO).times(a));
                motors[i].getSimState()
                        .setRotorAcceleration(sim[0].getAngularAcceleration().times(GEAR_RATIO).times(a));
            }
        } else {
            for (int i = 0; i < sim.length; i++) {
                sim[i].setInputVoltage(motors[i].getMotorVoltage().getValueAsDouble());
                sim[i].update(0.020);

                int a = TALON_CONSTANTS[i].INVERT == InvertedValue.CounterClockwise_Positive ? 1 : -1;

                motors[i].getSimState().setRotorVelocity(sim[i].getAngularVelocity().times(GEAR_RATIO).times(a));
                motors[i].getSimState()
                        .setRotorAcceleration(sim[i].getAngularAcceleration().times(GEAR_RATIO).times(a));
            }
        }
    }

    /**
     * @param talonConstants  An array containing a {@code TalonConstants} for each
     *                        motor in the subsystem. If {@code areFollowers} is set
     *                        to {@code true}, the first motor in the array will be
     *                        the master controller.
     * @param areFollowers    A {@code boolean} that describes whether the motors
     *                        are in a follower configuration or are controlled
     *                        independently. If they are mechanically linked, they
     *                        should be followers (denoted by returning
     *                        {@code true}).
     * @param name            A {@code String} object containing the name of the
     *                        subsystem. This will mainly be used for the purpose of
     *                        logging.
     * @param currentLimit    A {@code Current} object that represents the limit of
     *                        electrical current allowed per motor.
     * @param gearRatio       A {@code double} object that holds the gear ratio,
     *                        where greater than 1 is a reduction.
     * @param neutral         A {@code NeutralModeValue} object that describes the
     *                        system's behavior when no control is being applied.
     *                        Being set to {@code Coast} means it will keep moving,
     *                        and {@code Brake} will attempt to stop the system in a
     *                        no-control state.
     * @param canBus          A {@code CANBus} object that holds the CanBus the
     *                        subsystem is connected to.
     * @param simMotorPlant   A DCMotor plant using
     *                        {@code DCMotor.getKrakenX60Foc(x)} or
     *                        {@code DCMotor.getKrakenX40Foc(x)}. Note
     *                        that if {@code areFollowers} is set to {@code false},
     *                        x should be 1 motor because each motor will get a
     *                        sim, otherwise x should be equal to the amount of
     *                        motors.
     * @param moi             The moment of inertia of the flywheel.
     * @param tuningConstants A {@code TuningConstants} object representing the PID
     *                        and feedforward constants
     */
    public FinishedFlywheel(TalonConstants[] talonConstants, boolean areFollowers, String name, Current currentLimit,
            double gearRatio, NeutralModeValue neutral, CANBus canBus, DCMotor simMotorPlant, MomentOfInertia moi,
            TuningConstants tuningConstants) {
        TALON_CONSTANTS = talonConstants;
        ARE_FOLLOWERS = areFollowers;
        NAME = name;
        CURRENT_LIMIT = currentLimit;
        GEAR_RATIO = gearRatio;
        NEUTRAL = neutral;
        CANBUS = canBus;
        SIM_MOTOR_PLANT = simMotorPlant;
        MOI = moi;
        K = tuningConstants;

        followerRequest = new StrictFollower(TALON_CONSTANTS[0].CAN_ID);
        motors = new TalonFX[TALON_CONSTANTS.length];
        sim = new FlywheelSim[ARE_FOLLOWERS ? 1 : TALON_CONSTANTS.length];

        createSims();
        configureMotors();
        logMotors();
    }

    private void createSims() {
        for (int i = 0; i < sim.length; i++) {
            sim[i] = new FlywheelSim(
                    LinearSystemId.createFlywheelSystem(
                            SIM_MOTOR_PLANT,
                            MOI.in(KilogramSquareMeters),
                            GEAR_RATIO),
                    SIM_MOTOR_PLANT);
        }
    }

    private void configureMotors() {

        config.CurrentLimits.StatorCurrentLimitEnable = RobotBase.isReal();
        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
        config.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT.in(Amps);
        config.MotorOutput.NeutralMode = NEUTRAL;
        config.Slot0.withKP(K.getkP()).withKI(K.getkI()).withKD(K.getkD()).withKG(K.getkG()).withKA(K.getkA())
                .withKS(K.getkS()).withKV(K.getkV());
        for (int i = 0; i < motors.length; i++) {
            motors[i] = new TalonFX(TALON_CONSTANTS[i].CAN_ID, CANBUS);
            config.MotorOutput.Inverted = TALON_CONSTANTS[i].INVERT;
            motors[i].getConfigurator().apply(config);

            if (i != 0 && ARE_FOLLOWERS) {
                motors[i].setControl(followerRequest);
            }
        }
    }

    private void logMotors() {
        int index = 0;
        for (TalonFX motor : motors) {
            LoggingManager.getInstance()
                    .addGroup(new LogGroup(String.join("/", "subsystems", NAME, TALON_CONSTANTS[index].SYSTEM_NAME),
                            LogProfiles.logTalonFX(() -> motor)));
            index++;
        }

        LoggingManager.getInstance()
                .addGroup(new LogGroup(String.join("/", "subsystems", NAME, "goalVelocity"),
                        LogProfiles.logMeasure(() -> goalVelocity)));
        LoggingManager.getInstance()
                .addGroup(new LogGroup(String.join("/", "subsystems", NAME, "currentVelocity"),
                        LogProfiles.logMeasure(() -> getVelocity())));
    }

    private void setMotorsControl(ControlRequest control) {
        motors[0].setControl(control);

        for (int i = 1; i < motors.length; i++) {
            motors[i].setControl(ARE_FOLLOWERS ? followerRequest : control);
        }
    }

    public void stop() {
        for (int i = 0; i < motors.length; i++) {
            motors[i].stopMotor();
        }
    }

    public Command stopCommand() {
        return runOnce(() -> stop());
    }
}
