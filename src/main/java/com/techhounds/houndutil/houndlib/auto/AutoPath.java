package com.techhounds.houndutil.houndlib.auto;

import edu.wpi.first.math.trajectory.Trajectory;

/**
 * A wrapper for a WPILib trajectory that includes a name.
 * 
 * @author dr
 */
public class AutoPath {
    private String name;
    private Trajectory trajectory;

    /**
     * Initializes the AutoPath.
     * 
     * @param name       the name of the AutoPath
     * @param trajectory the trajectory to use
     */
    public AutoPath(String name, Trajectory trajectory) {
        this.name = name;
        this.trajectory = trajectory;
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
    public Trajectory getTrajectory() {
        return trajectory;
    }
}
