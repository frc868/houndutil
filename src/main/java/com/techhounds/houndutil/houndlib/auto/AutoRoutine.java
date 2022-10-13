package com.techhounds.houndutil.houndlib.auto;

import java.util.ArrayList;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoRoutine {
    private String name;
    private Command command;
    private ArrayList<AutoPath> trajectories = new ArrayList<AutoPath>();

    /**
     * Initialize an AutoRoutine.
     * 
     * @param name         the name of the routine, this will be pushed to
     *                     Shuffleboard
     * @param command      the command to run
     * @param trajectories an ArrayList of trajectories associated with the command,
     *                     used for Shuffleboard visualization, make zero index the
     *                     first trajectory
     */
    public AutoRoutine(String name, Command command, ArrayList<AutoPath> trajectories) {
        this.name = name;
        this.command = command;
        this.trajectories = trajectories;
    }

    /**
     * Initialize an AutoRoutine.
     * 
     * @param name    the name of the routine, this will be pushed to Shuffleboard
     * @param command the command to run
     */
    public AutoRoutine(String name, Command command) {
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
     * Get the trajectories associated with this routine.
     * 
     * @return the trajectories associated with this routine
     */
    public ArrayList<AutoPath> getTrajectories() {
        return trajectories;
    }

}
