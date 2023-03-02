package com.techhounds.houndutil.houndauto;

import java.util.function.Supplier;

public class AutoRoutine {
    private String name;
    private Supplier<AutoTrajectoryCommand> command;

    /**
     * Initialize an AutoRoutine. Will grab the {@code PPAutoPath} from the
     * {@code AutoTrajectoryCommand}.
     * 
     * @param name    the name of the routine, this will be pushed to
     *                Shuffleboard
     * @param command the command to run, containing its trajectories
     */
    public AutoRoutine(String name, Supplier<AutoTrajectoryCommand> command) {
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
    public Supplier<AutoTrajectoryCommand> getCommand() {
        return command;
    }

}
