package com.techhounds.houndutil.houndlib;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class BallState {
    public Translation3d position;
    public Translation3d velocity;
    public Rotation3d orientation;
    public Translation3d omega; // rad/s

    public BallState(
            Translation3d position,
            Translation3d velocity,
            Rotation3d orientation,
            Translation3d omega) {
        this.position = position;
        this.velocity = velocity;
        this.orientation = orientation;
        this.omega = omega;
    }
}