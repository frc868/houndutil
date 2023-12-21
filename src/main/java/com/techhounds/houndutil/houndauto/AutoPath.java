package com.techhounds.houndutil.houndauto;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;

/**
 * A wrapper for a set of PathPlanner paths,  that includes a name and can take in a
 * group of paths.
 * 
 * @author dr
 */
public class AutoPath {
    private String name;
    private List<PathPlannerPath> pathPlannerPaths;

    /**
     * Initializes the AutoPath.
     * 
     * @param name       the name of the AutoPath
     * @param trajectory the trajectory to use
     */
    public AutoPath(String name, List<PathPlannerPath> paths) {
        this.name = name;
        this.pathPlannerPaths = paths;
    }

    /**
     * Initializes the AutoPath.
     * 
     * @param name       the name of the AutoPath
     * @param trajectory the trajectory to use
     */
    public AutoPath(String name, PathPlannerPath... paths) {
        this(name, new ArrayList<PathPlannerPath>(Arrays.asList(paths)));
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
    public List<PathPlannerPath> getPaths() {
        return pathPlannerPaths;
    }

}
