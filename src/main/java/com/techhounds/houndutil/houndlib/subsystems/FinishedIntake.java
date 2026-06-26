package com.techhounds.houndutil.houndlib.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.techhounds.houndutil.houndlog.SignalManager;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@LoggedObject
/**
 * Finished intake mechanism.
 */
public class FinishedIntake extends SubsystemBase{

    public final FinishedTalonSystem[] TALON_INFO;
    public final boolean ARE_FOLLOWERS;
    public final String NAME;
    public final Voltage INTAKE_VOLTAGE;
    public final Voltage REVERSE_VOLTAGE;
    public final boolean IS_ONE_SIM;
    public final Current CURRENT_LIMIT;
    public final double GEAR_RATIO;
    public final NeutralModeValue NEUTRAL;
    public final DCMotor MOTOR_GEARBOX_REPR;
    public final MomentOfInertia MOMENT_OF_INERTIA;
    public final CANBus CANBUS;

    public final TalonFXConfiguration config = new TalonFXConfiguration();
    @Log public final TalonFX[] motors;
    @Log public final FlywheelSim[] sim;
    @Log public final StrictFollower followerRequest;
    @Log public final VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(true).withUseTimesync(true);
    @Log public final BaseStatusSignal[] voltageSignal;

    /**
     * Creates a command that runs the rollers of the intake in the direction that
     * will index the object (likely a game piece) into the robot.
     * 
     * @return the command
     */
    public Command runRollersCommand(){
        return runEnd(
            () -> setMotorsControl(voltageRequest.withOutput(INTAKE_VOLTAGE)),
            () -> stop()
        ).withName(NAME + ".runRollers");
    }

    /**
     * Creates a command that runs the rollers of the intake in the direction that
     * will eject the object (likely a game piece) out of the robot.
     * 
     * @return the command
     */
    public Command reverseRollersCommand(){
        return runEnd(
            () -> setMotorsControl(voltageRequest.withOutput(REVERSE_VOLTAGE)),
            () -> stop()
        ).withName(NAME + ".reverseRollers");
    }

    /**
     * Creates a command that stops the rollers of the intake.
     * 
     * @return the command
     */
    public Command stopRollersCommand(){
        return runOnce(
            () -> stop()
        ).withName(NAME + ".stopRollers");
    }

    @Override
    public void simulationPeriodic() {
        for(int i = 0; i < sim.length; i++){
            int a = TALON_INFO[i].INVERT == InvertedValue.CounterClockwise_Positive ? 1 : -1 ;

            sim[i].setInputVoltage(motors[i].getMotorVoltage().getValueAsDouble());
            sim[i].update(0.020);


            motors[i].getSimState().setRotorVelocity(sim[i].getAngularVelocity().div(GEAR_RATIO).times(a));
            motors[i].getSimState().setRotorAcceleration(sim[i].getAngularAcceleration().div(GEAR_RATIO).times(a));
        }
    }
    

    public FinishedIntake(FinishedTalonSystem[] talonInfo, boolean areFollowers, String name, Voltage intakeVoltage, Voltage reverseVoltage, boolean isOneSim, Current currentLimit, double gearRatio, NeutralModeValue neutral, DCMotor motorGearboxRepr, MomentOfInertia momentOfInertia, CANBus bus){
        TALON_INFO = talonInfo;
        ARE_FOLLOWERS = areFollowers;
        NAME = name;
        INTAKE_VOLTAGE = intakeVoltage;
        REVERSE_VOLTAGE = reverseVoltage;
        IS_ONE_SIM = isOneSim;
        CURRENT_LIMIT = currentLimit;
        GEAR_RATIO = gearRatio;
        NEUTRAL = neutral;
        MOTOR_GEARBOX_REPR = motorGearboxRepr;
        MOMENT_OF_INERTIA = momentOfInertia;
        CANBUS = bus;

        followerRequest = new StrictFollower(TALON_INFO[0].CAN_ID);
        motors = new TalonFX[TALON_INFO.length];
        sim = new FlywheelSim[ARE_FOLLOWERS || IS_ONE_SIM ? 1 : TALON_INFO.length];
        voltageSignal = new BaseStatusSignal[TALON_INFO.length];

        createSims();
        configureMotors();
        handleSignals();
    }

    private void createSims(){
        for(int i = 0; i < sim.length; i++){
            sim[i] = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                    MOTOR_GEARBOX_REPR,
                    MOMENT_OF_INERTIA.in(KilogramSquareMeters),
                    GEAR_RATIO),
            MOTOR_GEARBOX_REPR);
        }
    }

    private void configureMotors(){

        config.CurrentLimits.StatorCurrentLimitEnable = RobotBase.isReal();
        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
        config.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT.in(Amps);
        config.MotorOutput.NeutralMode = NEUTRAL;

        for(int i = 0; i < motors.length; i++){
            motors[i] = new TalonFX(TALON_INFO[i].CAN_ID, CANBUS);
            config.MotorOutput.Inverted = TALON_INFO[i].INVERT;
            motors[i].getConfigurator().apply(config);

            if(i != 0 && ARE_FOLLOWERS){
                motors[i].setControl(followerRequest);
            }
        }
    }

    private void setMotorsControl(ControlRequest control){
        motors[0].setControl(control);
        if(!ARE_FOLLOWERS){
            for(int i = 1; i < motors.length; i++){
                motors[i].setControl(control);
            }
        }
    }

    private void handleSignals(){
        for(int i = 0; i < voltageSignal.length; i++){
            voltageSignal[i] = motors[i].getMotorVoltage();
        }
        SignalManager.register(CANBUS.getName(), voltageSignal);
    }

    private void stop(){
        for(int i = 0; i < motors.length; i++){
            motors[i].stopMotor();
        }
    }
}
