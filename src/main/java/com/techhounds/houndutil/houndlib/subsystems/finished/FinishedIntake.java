package com.techhounds.houndutil.houndlib.subsystems.finished;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.techhounds.houndutil.houndlog.LogProfiles;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.loggers.LogGroup;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//NOT a logged object (kinda)
/**
 * An intake mechanism.
 */
public abstract class FinishedIntake extends SubsystemBase implements FinishedSubsystemBase {

    @Override
    public final double[] getTuningConstants() {
        throw new UnsupportedOperationException("getTuningConstants() is not used in an intake.");
    }

    private final FinishedTalonSystem[] TALON_INFO;
    private final boolean ARE_FOLLOWERS;
    private final String NAME;
    private final Current CURRENT_LIMIT;
    private final double GEAR_RATIO;
    private final NeutralModeValue NEUTRAL;
    private final CANBus CANBUS;

    private final TalonFXConfiguration config = new TalonFXConfiguration();
    private final TalonFX[] motors;
    private final FlywheelSim[] sim;
    private final StrictFollower followerRequest;
    private final VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(true).withUseTimesync(true);

    /**
     * Creates a command that runs the rollers of the intake at a specific voltage.
     * A negative voltage moves it in the opposite direction.
     * 
     * @return the command
     */
    public Command runRollersCommand(Voltage voltage) {
        return runEnd(
                () -> setMotorsControl(voltageRequest.withOutput(voltage)),
                () -> stop()).withName(NAME + ".runRollers");
    }

    /**
     * Creates a command that stops the rollers of the intake.
     * 
     * @return the command
     */
    public Command stopRollersCommand() {
        return runOnce(
                () -> stop()).withName(NAME + ".stopRollers");
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

    public FinishedIntake() {
        TALON_INFO = getTalonInfo();
        ARE_FOLLOWERS = getAreFollowers();
        NAME = getSubsystemName();
        CURRENT_LIMIT = getCurrentLimit();
        GEAR_RATIO = getGearRatio();
        NEUTRAL = getNeutral();
        CANBUS = getCanBus();

        followerRequest = new StrictFollower(TALON_INFO[0].CAN_ID);
        motors = new TalonFX[TALON_INFO.length];
        sim = new FlywheelSim[ARE_FOLLOWERS ? 1 : TALON_INFO.length];

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

    public void setMotorsControl(ControlRequest control) {
        motors[0].setControl(control);
        if (!ARE_FOLLOWERS) {
            for (int i = 1; i < motors.length; i++) {
                motors[i].setControl(control);
            }
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
