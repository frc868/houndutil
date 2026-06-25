package com.techhounds.houndutil.houndlib.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@LoggedObject
/**
 * Finished intake mechanism.
 */
public class FinishedIntake extends SubsystemBase{

    public final FinishedTalonSystem[] TALON_INFO;

    public final TalonFX[] MOTORS;

    /**
     * Creates a command that runs the rollers of the intake in the direction that
     * will index the object (likely a game piece) into the robot.
     * 
     * @return the command
     */
    public Command runRollersCommand(){
        return run
    }

    /**
     * Creates a command that runs the rollers of the intake in the direction that
     * will eject the object (likely a game piece) out of the robot.
     * 
     * @return the command
     */
    public Command reverseRollersCommand(){

    }

    /**
     * Creates a command that stops the rollers of the intake.
     * 
     * @return the command
     */
    public Command stopRollersCommand(){

    }

    public FinishedIntake(FinishedTalonSystem[] talonSystems){
        TALON_INFO = talonSystems;
        MOTORS = new TalonFX[TALON_INFO.length];
        for(int i = 0; i < MOTORS.length; i++){
            MOTORS[i] = new TalonFX(TALON_INFO[i].CAN_ID, TALON_INFO[i].CANBUS);
        }
    }
}
