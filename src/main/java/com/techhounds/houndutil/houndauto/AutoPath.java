package com.techhounds.houndutil.houndauto;

import java.util.ArrayList;
import java.util.Arrays;

import com.pathplanner.lib.PathPlannerTrajectory;

/**
 * A wrapper for a PathPlanner trajectory that includes a name and can take in a
 * group of paths.
 * 
 * @author dr
 */
public class AutoPath {
    private String name;
    private ArrayList<PathPlannerTrajectory> trajectories = new ArrayList<PathPlannerTrajectory>();

    /**
     * Initializes the AutoPath.
     * 
     * @param name       the name of the AutoPath
     * @param trajectory the trajectory to use
     */
    public AutoPath(String name, ArrayList<PathPlannerTrajectory> trajectories) {
        this.name = name;
        this.trajectories = trajectories;
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
     * Gets the trajectory associated with this AutoPath.
     * 
     * @return the trajectory associated with this AutoPath
     */
    public ArrayList<PathPlannerTrajectory> getTrajectories() {
        return trajectories;
    }
}
