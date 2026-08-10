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

/**
 * An intake mechanism. To use, after creating your subsystem file and class,
 * type
 * {@code extends FinishedIntake} after your class name (and before the
 * bracket).
 * <p>
 * All you need to do in order to make an Intake is call super() in the
 * intakes's constructor, and the FinishedIntake will handle the logic.
 * <p>
 * If you want to add custom commands and/or methods, make sure to utilize the
 * methods built
 * into the intake.
 * <p>
 * <h3>Built in methods:</h3>
 * {@code stop()}<p>
 * {@code stopCommand()}<p>
 * {@code runRollersCommand(Voltage)}<p>
 * {@code setMotorsControl(ControlRequest)}
 * 
 */
public abstract class FinishedIntake extends SubsystemBase implements FinishedJavadocs <AngularVelocity> {

    private final TalonConstants[] TALON_CONSTANTS;
    private final boolean ARE_FOLLOWERS;
    private final String NAME;
    private final Current CURRENT_LIMIT;
    private final double GEAR_RATIO;
    private final NeutralModeValue NEUTRAL;
    private final CANBus CANBUS;

    private final DCMotor SIM_MOTOR_PLANT;
    private final MomentOfInertia MOI;

    private final TalonFXConfiguration config = new TalonFXConfiguration();
    private final TalonFX[] motors;
    private final FlywheelSim[] sim;
    private final StrictFollower followerRequest;
    protected final VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(true).withUseTimesync(true);

    @Override
    public Command runRollersCommand(Voltage voltage) {
        return runEnd(
                () -> setMotorsControl(voltageRequest.withOutput(voltage)),
                () -> stop()).withName(NAME + ".runRollers");
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
                int a = TALON_CONSTANTS[i].INVERT == InvertedValue.CounterClockwise_Positive ? 1 : -1;

                sim[i].setInputVoltage(motors[i].getMotorVoltage().getValueAsDouble());
                sim[i].update(0.020);

                motors[i].getSimState().setRotorVelocity(sim[i].getAngularVelocity().times(GEAR_RATIO).times(a));
                motors[i].getSimState()
                        .setRotorAcceleration(sim[i].getAngularAcceleration().times(GEAR_RATIO).times(a));
            }
        }
    }

    /**
     * talonConstants An array containing a {@code TalonConstants} for each motor in
     * the subsystem. If {@code areFollowers} is set to {@code true}, the first
     * motor in the array will be the master controller.
     * 
     * @param areFollowers  A {@code boolean} that describes whether the motors are
     *                      in a follower configuration or are controlled
     *                      independently. If they are mechanically linked, they
     *                      should be followers (denoted by returning {@code true}).
     * @param name          A {@code String} object containing the name of the
     *                      subsystem. This will mainly be used for the purpose of
     *                      logging.
     * @param currentLimit  A {@code Current} object that represents the limit of
     *                      electrical current allowed per motor.
     * @param gearRatio     A {@code double} object that holds the gear ratio, where
     *                      greater than 1 is a reduction.
     * @param neutral       A {@code NeutralModeValue} object that describes the
     *                      system's behavior when no control is being applied.
     *                      Being
     *                      set to {@code Coast} means it will keep moving, and
     *                      {@code Brake} will attempt to stop the system in a
     *                      no-control state.
     * @param canBus        A {@code CANBus} object that holds the CanBus the
     *                      subsystem is connected to.
     * @param simMotorPlant A DCMotor plant using
     *                      {@code DCMotor.getKrakenX60Foc(x)} or
     *                      {@code DCMotor.getKrakenX40Foc(x)}. Note
     *                      that if {@code areFollowers} is set to {@code false},
     *                      x should be 1 motor because each motor will get a
     *                      sim, otherwise x should be equal to the amount of
     *                      motors.
     * @param moi           The moment of inertia of the flywheel.
     */
    public FinishedIntake(TalonConstants[] talonConstants, boolean areFollowers, String name, Current currentLimit,
            double gearRatio, NeutralModeValue neutral, CANBus canBus, DCMotor simMotorPlant, MomentOfInertia moi) {
        TALON_CONSTANTS = talonConstants;
        ARE_FOLLOWERS = areFollowers;
        NAME = name;
        CURRENT_LIMIT = currentLimit;
        GEAR_RATIO = gearRatio;
        NEUTRAL = neutral;
        CANBUS = canBus;
        SIM_MOTOR_PLANT = simMotorPlant;
        MOI = moi;

        followerRequest = new StrictFollower(TALON_CONSTANTS[0].CAN_ID);
        motors = new TalonFX[TALON_CONSTANTS.length];
        sim = new FlywheelSim[ARE_FOLLOWERS ? 1 : TALON_CONSTANTS.length];

        createSims();
        configureMotors();
        logMotors();
    }

    private void createSims() {
        for (int i = 0; i < sim.length; i++) {
            sim[i] = sim[i] = new FlywheelSim(
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
    }

    @Override
    public void setMotorsControl(ControlRequest control) {
        motors[0].setControl(control);

        for (int i = 1; i < motors.length; i++) {
            motors[i].setControl(ARE_FOLLOWERS ? followerRequest : control);
        }
    }

    @Override
    public void stop() {
        for (int i = 0; i < motors.length; i++) {
            motors[i].stopMotor();
        }
    }

    @Override
    public Command stopCommand() {
        return runOnce(() -> stop());
    }

    // TODO maybe add a getVelocity() method but idk
}
