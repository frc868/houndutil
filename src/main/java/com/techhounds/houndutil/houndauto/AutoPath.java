package com.techhounds.houndutil.houndauto;

import java.util.ArrayList;
import java.util.Arrays;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * A wrapper for a PathPlanner trajectory that includes a name and can take in a
 * group of paths.
 * 
 * @author dr
 */
public class AutoPath {
    private String name;
    private ArrayList<PathPlannerTrajectory> blueTrajectories = new ArrayList<PathPlannerTrajectory>();
    private ArrayList<PathPlannerTrajectory> redTrajectories = new ArrayList<PathPlannerTrajectory>();

    /**
     * Initializes the AutoPath.
     * 
     * @param name       the name of the AutoPath
     * @param trajectory the trajectory to use
     */
    public AutoPath(String name, ArrayList<PathPlannerTrajectory> trajectories) {
        this.name = name;
        this.blueTrajectories = trajectories;

        for (PathPlannerTrajectory trajectory : trajectories) {
            redTrajectories.add(TrajectoryReflector.reflectiveTransformTrajectory(trajectory, 16.54));
        }
    }

    /**
     * Initializes the AutoPath.
     * 
     * @param name       the name of the AutoPath
     * @param trajectory the trajectory to use
     */
    public AutoPath(String name, PathPlannerTrajectory... trajectories) {
        this(name, new ArrayList<PathPlannerTrajectory>(Arrays.asList(trajectories)));
    }

    /**
     * Gets the name of the AutoPath.
     * 
     * @return the name of the AutoPath
     */
    public String getName() {
        return name;
    }

    /**
     * Gets the trajectories associated with this AutoPath.
     * 
     * @return the trajectories associated with this AutoPath
     */
    public ArrayList<PathPlannerTrajectory> getTrajectories() {
        if (DriverStation.getAlliance() == Alliance.Red) {
            return redTrajectories;
        } else {
            return blueTrajectories;
        }
    }
}
