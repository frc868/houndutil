package com.techhounds.houndutil.houndlib.auto.trajectoryloader;

public class TrajectorySettings {
    public String name;
    public double maxVelocity = 1.0;
    public double maxAcceleration = 1.0;
    public boolean isReversed = false;

    public TrajectorySettings(String name) {
        this.name = name;
    }

    public TrajectorySettings withMaxVelocity(double maxVelocity) {
        this.maxVelocity = maxVelocity;
        return this;
    }

    public TrajectorySettings withMaxAcceleration(double maxAcceleration) {
        this.maxAcceleration = maxAcceleration;
        return this;
    }

    public TrajectorySettings withReversed(boolean isReversed) {
        this.isReversed = isReversed;
        return this;
    }
}
