package com.techhounds.houndutil.houndlib.subsystems;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Base scaffolding for an intake mechanism.
 */
public interface BaseIntake {
    /**
     * Creates a command that runs the rollers of the intake in the direction that
     * will index the object (likely a game piece) into the robot.
     * 
     * @return the command
     */
    public Command runRollersCommand();

    /**
     * Creates a command that runs the rollers of the intake in the direction that
     * will eject the object (likely a game piece) out of the robot.
     * 
     * @return the command
     */
    public Command reverseRollersCommand();
}
