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
import com.techhounds.houndutil.houndlog.LogProfiles;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.loggers.LogGroup;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

//NOT a logged object (kinda)
/**
 * A flywheel mechanism.
 */
public abstract class FinishedFlywheel extends SubsystemBase implements FinishedSubsystemBase {

    private final FinishedTalonSystem[] TALON_INFO;
    private final boolean ARE_FOLLOWERS;
    private final String NAME;
    private final Current CURRENT_LIMIT;
    private final double GEAR_RATIO;
    private final NeutralModeValue NEUTRAL;
    private final CANBus CANBUS;
    private final double[] K;

    private AngularVelocity goalVelocity = RotationsPerSecond.zero();
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
            total = total + motor.getVelocity().getValue().abs(RotationsPerSecond);
            i++;
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
                int a = TALON_INFO[i].INVERT == InvertedValue.CounterClockwise_Positive ? 1 : -1;

                motors[i].getSimState().setRotorVelocity(sim[0].getAngularVelocity().div(GEAR_RATIO).times(a));
                motors[i].getSimState().setRotorAcceleration(sim[0].getAngularAcceleration().div(GEAR_RATIO).times(a));
            }
        } else {
            for (int i = 0; i < sim.length; i++) {
                int a = TALON_INFO[i].INVERT == InvertedValue.CounterClockwise_Positive ? 1 : -1;

                sim[i].setInputVoltage(motors[i].getMotorVoltage().getValueAsDouble());
                sim[i].update(0.020);

                motors[i].getSimState().setRotorVelocity(sim[i].getAngularVelocity().div(GEAR_RATIO).times(a));
                motors[i].getSimState().setRotorAcceleration(sim[i].getAngularAcceleration().div(GEAR_RATIO).times(a));
            }
        }
    }

    public FinishedFlywheel() {
        TALON_INFO = getTalonInfo();
        ARE_FOLLOWERS = getAreFollowers();
        NAME = getName();
        CURRENT_LIMIT = getCurrentLimit();
        GEAR_RATIO = getGearRatio();
        NEUTRAL = getNeutral();
        CANBUS = getCanBus();
        K = getTuningConstants();

        followerRequest = new StrictFollower(TALON_INFO[0].CAN_ID);
        motors = new TalonFX[TALON_INFO.length];
        sim = new FlywheelSim[ARE_FOLLOWERS ? 1 : TALON_INFO.length];

        LoggingManager.getInstance()
                    .addGroup(new LogGroup(String.join("/", "subsystems", NAME, "goalVelocity"),
                            LogProfiles.logMeasure(() -> goalVelocity)));

        createSims();
        configureMotors();
        logMotors();
    }

    // TODO make sure sim is doing what it is supposed to do
    private void createSims() {
        for (int i = 0; i < sim.length; i++) {
            sim[i] = getFlywheelSim();
        }
    }

    private void configureMotors() {

        config.CurrentLimits.StatorCurrentLimitEnable = RobotBase.isReal();
        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
        config.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT.in(Amps);
        config.MotorOutput.NeutralMode = NEUTRAL;
        config.Slot0.withKP(1.0);

        for (int i = 0; i < motors.length; i++) {
            motors[i] = new TalonFX(TALON_INFO[i].CAN_ID, CANBUS);
            config.MotorOutput.Inverted = TALON_INFO[i].INVERT;
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
                    .addGroup(new LogGroup(String.join("/", "subsystems", NAME, TALON_INFO[index].SYSTEM_NAME),
                            LogProfiles.logTalonFX(() -> motor)));
            index++;
        }
    }

    private void setMotorsControl(ControlRequest control) {
        motors[0].setControl(control);
        if (!ARE_FOLLOWERS) {
            for (int i = 1; i < motors.length; i++) {
                motors[i].setControl(control);
            }
        }
    }

    private void stop() {
        for (int i = 0; i < motors.length; i++) {
            motors[i].stopMotor();
        }
    }

}
