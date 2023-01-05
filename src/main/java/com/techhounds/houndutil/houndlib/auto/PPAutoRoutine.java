package com.techhounds.houndutil.houndlib.auto;

import edu.wpi.first.wpilibj2.command.Command;

public class PPAutoRoutine {
    private String name;
    private Command command;
    private PPAutoPath autoPath;

    /**
     * Initialize an AutoRoutine. Will grab the {@code AutoPath} from the
     * {@code AutoTrajectoryCommand}.
     * 
     * @param name    the name of the routine, this will be pushed to
     *                Shuffleboard
     * @param command the command to run, containing its trajectories
     */
    public PPAutoRoutine(String name, PPAutoTrajectoryCommand command) {
        this.name = name;
        this.command = command;
        this.autoPath = command.getAutoPath();
    }

    /**
     * Initialize an AutoRoutine.
     * 
     * @param name     the name of the routine, this will be pushed to
     *                 Shuffleboard
     * @param command  the command to run
     * @param autoPath the trajectories associated with the command,
     *                 used for Shuffleboard visualization
     */
    public PPAutoRoutine(String name, Command command, PPAutoPath autoPath) {
        this.name = name;
        this.command = command;
        this.autoPath = autoPath;
    }

    /**
     * Initialize an AutoRoutine.
     * 
     * @param name    the name of the routine, this will be pushed to Shuffleboard
     * @param command the command to run
     */
    public PPAutoRoutine(String name, Command command) {
        this.name = name;
        this.command = command;
    }

    public String getName() {
        return name;
    }

    /**
     * Get the command associated with this autonomous routine.
     * 
     * @return the command associated with this autonomous routine
     */
    public Command getCommand() {
        return command;
    }

    /**
     * Get the {@code autoPath} associated with this routine.
     * 
     * @return the autoPaths associated with this routine
     */
    public PPAutoPath getAutoPath() {
        return autoPath;
    }

}
