package com.techhounds.houndutil.houndlib.auto.trajectoryloader;

import java.util.ArrayList;

import com.pathplanner.lib.PathConstraints;

public class TrajectorySettings {
    public String name;
    public double maxVelocity = 1.0;
    public double[] maxVelocities;
    public double maxAcceleration = 1.0;
    public double[] maxAccelerations;
    public boolean isReversed = false;

    public TrajectorySettings(String name) {
        this.name = name;
    }

    /**
     * Sets the
     * 
     * @param maxVelocity
     * @param maxVelocities
     * @return
     */
    public TrajectorySettings withMaxVelocity(double maxVelocity, double... maxVelocities) {
        this.maxVelocity = maxVelocity;
        this.maxVelocities = maxVelocities;
        return this;
    }

    public TrajectorySettings withMaxAcceleration(double maxAcceleration, double... maxAccelerations) {
        this.maxAcceleration = maxAcceleration;
        this.maxAccelerations = maxAccelerations;
        return this;
    }

    public ArrayList<PathConstraints> getConstraints() {
        ArrayList<PathConstraints> constraints = new ArrayList<PathConstraints>();
        constraints.add(new PathConstraints(maxVelocity, maxAcceleration));

        if (maxVelocities != null && maxAccelerations != null) {
            if (maxVelocities.length != maxAccelerations.length) {
                throw new IllegalArgumentException("maxVelocities and maxAccelerations must have the same length.");
            }

            if (maxVelocities.length > 0) {
                for (int i = 0; i < maxVelocities.length; i++) {
                    constraints.add(new PathConstraints(maxVelocities[i], maxAccelerations[i]));
                }
            }
        }

        return constraints;
    }
}
