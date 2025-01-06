package com.techhounds.houndutil.houndauto;

import java.util.List;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A dataclass that stores additional information about an autonomous routine.
 * Used by {@link AutoManager}.
 */
public class AutoRoutine {
    private String name;
    private Command command;
    private List<PathPlannerPath> pathPlannerPaths = List.of();
    private Pose2d initialPose;

    /**
     * Creates an AutoRoutine. Use for auto routines that do not use trajectories or
     * localization.
     * 
     * @param name    the name of the routine, this will be pushed to
     *                NetworkTables
     * @param command the command to run on auto enable
     */
    public AutoRoutine(String name, Command command) {
        this.name = name;
        this.command = command;
    }

    /**
     * Creates an AutoRoutine.
     * 
     * @param name             the name of the routine, this will be pushed to
     *                         NetworkTables
     * @param commandSupplier  the command to run on auto enable
     * @param pathPlannerPaths a list of paths that the command contains
     * @param initialPose      the initial blue pose of the path, used to reset
     *                         odometry
     */
    public AutoRoutine(String name, Command command, List<PathPlannerPath> pathPlannerPaths, Pose2d initialPose) {
        this.name = name;
        this.command = command;
        this.pathPlannerPaths = pathPlannerPaths;
        this.initialPose = initialPose;
    }

    /**
     * Gets the name of the routine.
     * 
     * @return the name of the routine
     */
    public String getName() {
        return name;
    }

    /**
     * Gets the command associated with the routine.
     * 
     * @return the command for the routine
     */
    public Command getCommand() {
        return command;
    }

    /**
     * Gets the paths associated with the routine.
     * 
     * @return the paths for the routine (empty if no paths)
     */
    public List<PathPlannerPath> getPathPlannerPaths() {
        return pathPlannerPaths;
    }

    /**
     * Gets the initial pose for the routine. When the DriverStation is set to the
     * red alliance, this is reflected over a field length of 16.54m.
     * 
     * @return the initial pose for the routine, or the pose at (0,0) if no
     *         trajectories are used.
     */
    public Pose2d getInitialPose() {
        if (pathPlannerPaths.size() > 0) {
            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red)
                return Reflector.reflectPose2d(initialPose, 16.54);
            else
                return initialPose;
        } else {
            return new Pose2d();
        }
    }

}
