package com.techhounds.houndutil.houndlib.subsystems.finished;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.techhounds.houndutil.houndlib.Utils;
import com.techhounds.houndutil.houndlog.LogProfiles;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.loggers.LogGroup;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;

//TODO fix this when done
/**
 * A linear mechanism. To use, after creating your subsystem file and class,
 * type
 * {@code extends FinishedPivot} after your class name (and before the
 * bracket).
 * <p>
 * All you need to do in order to make a linear mechanism is call super() in the
 * linear mechanism's constructor, and the FinishedPivot will handle
 * the logic.
 * <p>
 * If you want to add custom commands, make sure to utilize the commands built
 * into the mechanism.
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
public abstract class FinishedPivot extends SubsystemBase {

    private final TalonConstants[] TALON_CONSTANTS;
    private final boolean ARE_FOLLOWERS;
    private final String NAME;
    private final Current CURRENT_LIMIT;
    private final double GEAR_RATIO;
    private final NeutralModeValue NEUTRAL;
    private final CANBus CANBUS;

    private final DCMotor SIM_MOTOR_PLANT;
    private final MomentOfInertia MOI;
    private final Distance ARM_LENGTH;
    private final AngularVelocity MAX_VELOCITY;
    private final AngularAcceleration MAX_ACCELERATION;
    private final Angle MIN_POSITION;
    private final Angle MAX_POSITION;
    private final Angle TOLERANCE;

    private final double[] K;

    private Angle goalPosition;
    private final TalonFXConfiguration config = new TalonFXConfiguration();
    private final TalonFX[] motors;
    private final ArrayList<SingleJointedArmSim> sim;
    private final StrictFollower followerRequest;
    private final VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(true).withUseTimesync(true);
    private final DynamicMotionMagicVoltage positionRequest;

    // TODO add "resetPosition()" allegedly

    /**
     * Gets the position of the mechanism. 0 should be at the lowest movement point,
     * and the position should increase as the mechanism moves up.
     * 
     * @return the position of the mechanism, in meters
     */
    public Angle getPosition() {
        double total = 0.0;
        int i = 0;
        for (TalonFX motor : motors) {
            if (motor != null) {
                total = total + motor.getPosition().getValue().in(Degrees);
                i++;
            }
        }
        if (i == 0) {
            return Degrees.zero();
        }

        total /= i;

        return Degrees.of(total);
    }

    /**
     * Creates a command that continuously applies voltage to the motor controllers
     * to move them to the currently set goal.
     * 
     * @return the command
     */
    public Command moveToCurrentGoalCommand() {
        return run(() -> {
            setMotorsControl(positionRequest.withPosition(
                    Utils.applySoftStops(goalPosition, MIN_POSITION, MAX_POSITION)));
        }).withName(NAME + ".moveToCurrentGoal");
    }

    /**
     * Creates a command that sets the current goal position to the setpoint, and
     * cancels once the mechanism has reached that goal.
     * 
     * @apiNote use {@code moveToCurrentGoalCommand()} internally to avoid code
     *          duplication
     * 
     * @param goalPositionSupplier a supplier of a position to move to
     * @return the command
     */
    public Command moveToArbitraryPositionCommand(Supplier<Angle> goalPositionSupplier) {
        return Commands.runOnce(() -> {
            goalPosition = goalPositionSupplier.get();
        }).andThen(moveToCurrentGoalCommand()).until(() -> getPosition().isNear(goalPosition, TOLERANCE))
                .withName(NAME + ".moveToPosition");
    }

    /**
     * Creates a command that sets the current goal position to the setpoint plus
     * the delta (if a delta of 0.1 is set, the linear mechanism should move up
     * 10cm), and
     * cancels once the mechanism has reached that goal.
     * 
     * 
     * @param delta a supplier of a delta to move
     * @return the command
     */
    public Command movePositionDeltaCommand(Supplier<Angle> delta) {
        return Commands.runOnce(() -> {
            goalPosition = getPosition().plus(delta.get());
        }).andThen(moveToCurrentGoalCommand()).until(() -> getPosition().isNear(goalPosition, TOLERANCE))
                .withName(NAME + ".moveToPositionDelta");
    }

    /**
     * Explicit function to set the voltage of the motors attached to the elevator,
     * should handle safeties and clamping here.
     * 
     * @param voltage the voltage to apply to the motors, [-12, 12]
     */
    public void setVoltage(Voltage voltage) {
        setMotorsControl(voltageRequest
                .withOutput(Utils.applySoftStops(Volts.of(MathUtil.clamp(voltage.in(Volts), -12, 12)), getPosition(),
                        MIN_POSITION, MAX_POSITION)));

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
                    goalPosition = getPosition();
                }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming).withName(NAME + ".coastMotors");
    }

    @Override
    public void simulationPeriodic() {
        if (ARE_FOLLOWERS) {
            sim.get(0).setInput(motors[0].getSimState().getMotorVoltage());
            sim.get(0).update(0.020);

            for (int i = 0; i < motors.length; i++) {
                int a = TALON_CONSTANTS[i].INVERT == InvertedValue.CounterClockwise_Positive ? 1 : -1;

                motors[i].getSimState().setRawRotorPosition(
                        sim.get(0).getAngleRads() / (2 * Math.PI) * GEAR_RATIO * a);
                motors[i].getSimState()
                        .setRotorVelocity(sim.get(0).getVelocityRadPerSec() / (2 * Math.PI) * GEAR_RATIO * a);
            }
        } else {
            for (int i = 0; i < sim.size(); i++) {
                int a = TALON_CONSTANTS[i].INVERT == InvertedValue.CounterClockwise_Positive ? 1 : -1;

                sim.get(i).setInput(motors[i].getSimState().getMotorVoltage());
                sim.get(i).update(0.020);

                motors[i].getSimState().setRawRotorPosition(
                        sim.get(i).getAngleRads() / (2 * Math.PI) * GEAR_RATIO * a);
                motors[i].getSimState()
                        .setRotorVelocity(sim.get(i).getVelocityRadPerSec() / (2 * Math.PI) * GEAR_RATIO * a);
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
     * @param tuningConstants An array of <b>exactly seven</b> {@code double}
     *                        objects representing the PID and feedforward constants
     *                        in the following order:
     *                        {@code &#123;kP, kI, kD, kG, kA, kS, kV&#125;}.
     */
    public FinishedPivot(TalonConstants[] talonConstants, boolean areFollowers, String name,
            Current currentLimit,
            double gearRatio, NeutralModeValue neutral, CANBus canBus, DCMotor simMotorPlant, MomentOfInertia moi,
            Distance armLength,
            AngularVelocity maxVelocity, AngularAcceleration maxAcceleration, Angle minPosition, Angle maxPosition,
            Angle tolerance,
            double[] tuningConstants) {
        TALON_CONSTANTS = talonConstants;
        ARE_FOLLOWERS = areFollowers;
        NAME = name;
        CURRENT_LIMIT = currentLimit;
        GEAR_RATIO = gearRatio;
        NEUTRAL = neutral;
        CANBUS = canBus;
        SIM_MOTOR_PLANT = simMotorPlant;
        MOI = moi;
        ARM_LENGTH = armLength;
        MAX_VELOCITY = maxVelocity;
        MAX_ACCELERATION = maxAcceleration;
        MIN_POSITION = minPosition;
        MAX_POSITION = maxPosition;
        TOLERANCE = tolerance;
        K = tuningConstants;

        goalPosition = MIN_POSITION;

        positionRequest = new DynamicMotionMagicVoltage(goalPosition, MAX_VELOCITY,
            MAX_ACCELERATION).withEnableFOC(true).withUseTimesync(true);

        if (tuningConstants.length != 7) {
            // TODO check how this can be printed to driverstation or something
            System.out.println("\033[1m"
                    + "WARNING: tuningConstants should have a length of 7. (Issued from FinishedPivot.java for subsystem: "
                    + NAME + ")" + "\033[0m");
        }

        followerRequest = new StrictFollower(TALON_CONSTANTS[0].CAN_ID);
        motors = new TalonFX[TALON_CONSTANTS.length];
        sim = new ArrayList<SingleJointedArmSim>();

        createSims();
        configureMotors();
        logMotors();
    }

    private void createSims() {
        for (int i = 0; i < motors.length; i++) {
            // check what starting angle sohuld be
            sim.add(new SingleJointedArmSim(SIM_MOTOR_PLANT, GEAR_RATIO, MOI.in(KilogramSquareMeters),
                    ARM_LENGTH.in(Meters), MIN_POSITION.in(Radians), MAX_POSITION.in(Radians), K[3] != 0.0,
                    Degrees.of(0.0).in(Radians)));
        }
    }

    private void configureMotors() {

        config.CurrentLimits.StatorCurrentLimitEnable = RobotBase.isReal();
        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
        config.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT.in(Amps);
        config.MotorOutput.NeutralMode = NEUTRAL;
        config.Slot0.withKP(K[0]).withKI(K[1]).withKD(K[2]).withKG(K[3]).withKA(K[4]).withKS(K[5]).withKV(K[6])
                .withGravityType(GravityTypeValue.Arm_Cosine);
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
                .addGroup(new LogGroup(String.join("/", "subsystems", NAME, "goalPosition"),
                        LogProfiles.logMeasure(() -> goalPosition)));
        LoggingManager.getInstance()
                .addGroup(new LogGroup(String.join("/", "subsystems", NAME, "currentPosition"),
                        LogProfiles.logMeasure(() -> getPosition())));
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
