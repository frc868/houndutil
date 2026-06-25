package com.techhounds.houndutil.houndlib.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@LoggedObject
/**
 * Finished intake mechanism.
 */
public class FinishedIntake extends SubsystemBase{

    public final FinishedTalonSystem[] TALON_INFO;
    public final boolean FOLLOWERS;

    public final TalonFXConfiguration config = new TalonFXConfiguration();
    public final TalonFX[] motors;

    /**
     * Creates a command that runs the rollers of the intake in the direction that
     * will index the object (likely a game piece) into the robot.
     * 
     * @return the command
     */
    public Command runRollersCommand(){
        return null;
    }

    /**
     * Creates a command that runs the rollers of the intake in the direction that
     * will eject the object (likely a game piece) out of the robot.
     * 
     * @return the command
     */
    public Command reverseRollersCommand(){
        return null;
    }

    /**
     * Creates a command that stops the rollers of the intake.
     * 
     * @return the command
     */
    public Command stopRollersCommand(){
        return null;
    }

    public FinishedIntake(FinishedTalonSystem[] talonSystems, boolean followers){
        TALON_INFO = talonSystems;
        FOLLOWERS = followers;

        motors = new TalonFX[TALON_INFO.length];

        configureMotors();
    }

    private void configureMotors(){

        config.CurrentLimits.StatorCurrentLimitEnable = RobotBase.isReal();
        for(int i = 0; i < motors.length; i++){
            motors[i] = new TalonFX(TALON_INFO[i].CAN_ID, TALON_INFO[i].CANBUS);
            config.Feedback.SensorToMechanismRatio = TALON_INFO[i].GEAR_RATIO;
            config.CurrentLimits.StatorCurrentLimit = TALON_INFO[i].CURRENT_LIMIT.in(Amps);
            config.MotorOutput.NeutralMode = TALON_INFO[i].NEUTRAL;
            config.MotorOutput.Inverted = TALON_INFO[i].INVERT;
            motors[i].getConfigurator().apply(config);
        }
    }
}
