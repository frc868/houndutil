package com.techhounds.houndutil.houndauto;

import java.util.List;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoRoutine {
    private String name;
    private Command command;
    private List<PathPlannerPath> pathPlannerPaths;
    private Pose2d initialPose;

    /**
     * Initialize an AutoRoutine.
     * 
     * @param name            the name of the routine, this will be pushed to
     *                        Shuffleboard
     * @param commandSupplier the command to run, containing its trajectories
     */
    public AutoRoutine(String name, Command command) {
        this.name = name;
        this.command = command;
    }

    public AutoRoutine(String name, Command command, List<PathPlannerPath> pathPlannerPaths, Pose2d initialPose) {
        this.name = name;
        this.command = command;
        this.pathPlannerPaths = pathPlannerPaths;
        this.initialPose = initialPose;
    }

    public String getName() {
        return name;
    }

    public Command getCommand() {
        return command;
    }

    public List<PathPlannerPath> getPathPlannerPaths() {
        return pathPlannerPaths;
    }

    public Pose2d getInitialPose() {
        if (pathPlannerPaths != null) {
            return initialPose;
        } else {
            return new Pose2d();
        }
    }

}
