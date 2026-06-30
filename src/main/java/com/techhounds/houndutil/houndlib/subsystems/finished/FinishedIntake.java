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
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;

//NOT a logged object (kinda)
/**
 * An intake mechanism.
 */
public class FinishedIntake extends FinishedSubsystemBase{

    public final FinishedTalonSystem[] TALON_INFO;
    public final boolean ARE_FOLLOWERS;
    public final String NAME;
    public final Voltage INTAKE_VOLTAGE;
    public final Voltage REVERSE_VOLTAGE;
    public final Current CURRENT_LIMIT;
    public final double GEAR_RATIO;
    public final NeutralModeValue NEUTRAL;
    public final DCMotor MOTOR_GEARBOX_REPR;
    public final MomentOfInertia MOMENT_OF_INERTIA;
    public final CANBus CANBUS;

    public final TalonFXConfiguration config = new TalonFXConfiguration();
    public final TalonFX[] motors;
    public final FlywheelSim[] sim;
    public final StrictFollower followerRequest;
    public final VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(true).withUseTimesync(true);

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
        if(ARE_FOLLOWERS){
            sim[0].setInputVoltage(motors[0].getMotorVoltage().getValueAsDouble());
            sim[0].update(0.020);

            for(int i = 0; i < motors.length; i++){
                int a = TALON_INFO[i].INVERT == InvertedValue.CounterClockwise_Positive ? 1 : -1 ;

                motors[i].getSimState().setRotorVelocity(sim[0].getAngularVelocity().div(GEAR_RATIO).times(a));
                motors[i].getSimState().setRotorAcceleration(sim[0].getAngularAcceleration().div(GEAR_RATIO).times(a));
            }
        }
        else{
            for(int i = 0; i < sim.length; i++){
                int a = TALON_INFO[i].INVERT == InvertedValue.CounterClockwise_Positive ? 1 : -1 ;

                sim[i].setInputVoltage(motors[i].getMotorVoltage().getValueAsDouble());
                sim[i].update(0.020);


                motors[i].getSimState().setRotorVelocity(sim[i].getAngularVelocity().div(GEAR_RATIO).times(a));
                motors[i].getSimState().setRotorAcceleration(sim[i].getAngularAcceleration().div(GEAR_RATIO).times(a));
            }
        }
    }
    
    /**
     * @param talonInfo A list of the FinishedTalonSystems inside the robot, each representing a motor.
     * @param areFollowers A boolean stating if the motors are followers, primarily used when mechanically connected.
     * @param name The name of the subsystem.
     * @param intakeVoltage The voltage applied to the intake when moving the normal direction.
     * @param reverseVoltage The voltage applied to the intake when moving the opposite direction.
     * @param currentLimit The limit of the amount of electrical current allowed in the motors
     * @param gearRatio The gear ratio between motor and the mechanism (>1 is a reduction).
     * @param neutral The behavior of the mechanism when no output is applied (brake or coast).
     * @param krackenType The type of the kracken.
     * @param momentOfInertia The MIO of the mechanism.
     * @param bus The canbus that the system is connected to.
     */
    public FinishedIntake(FinishedTalonSystem[] talonInfo, boolean areFollowers, String name, Voltage intakeVoltage, Voltage reverseVoltage, Current currentLimit, double gearRatio, NeutralModeValue neutral, KrackenType krackenType, MomentOfInertia momentOfInertia, CANBus bus){
        TALON_INFO = talonInfo;
        ARE_FOLLOWERS = areFollowers;
        NAME = name;
        INTAKE_VOLTAGE = intakeVoltage;
        REVERSE_VOLTAGE = reverseVoltage;
        CURRENT_LIMIT = currentLimit;
        GEAR_RATIO = gearRatio;
        NEUTRAL = neutral;
        MOMENT_OF_INERTIA = momentOfInertia;
        CANBUS = bus;

        if(krackenType.getInt() == 60){
            MOTOR_GEARBOX_REPR = DCMotor.getKrakenX60Foc(ARE_FOLLOWERS ? talonInfo.length : 1);
        }else if(krackenType.getInt() == 44){
            MOTOR_GEARBOX_REPR = DCMotor.getKrakenX44Foc(ARE_FOLLOWERS ? talonInfo.length : 1);
        }else{
            System.out.println("Needs to be 60 or 44");
            MOTOR_GEARBOX_REPR = DCMotor.getKrakenX44Foc(0);
        }

        followerRequest = new StrictFollower(TALON_INFO[0].CAN_ID);
        motors = new TalonFX[TALON_INFO.length];
        sim = new FlywheelSim[ARE_FOLLOWERS ? 1 : TALON_INFO.length];

        createSims();
        configureMotors();
        logMotors();
    }
    //TODO make sure sim is doing what it is supposed to do
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

    private void logMotors(){
        int index = 0;
        for(TalonFX motor: motors){
            LoggingManager.getInstance().addGroup(new LogGroup(String.join("/","subsystems" ,NAME ,TALON_INFO[index].SYSTEM_NAME), LogProfiles.logTalonFX(() -> motor)));
            index ++;
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

    private void stop(){
        for(int i = 0; i < motors.length; i++){
            motors[i].stopMotor();
        }
    }
}