package com.techhounds.houndutil.houndlib;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;

import java.util.ArrayList;
import java.util.List;

import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

@LoggedObject
public class BallSimulator {
    private static final double OUT_OF_BOUNDS_MARGIN = 1.0;

    private final BallConstants constants;
    private final List<BallState> balls = new ArrayList<>();

    private final double robotDt;
    private final double physicsDt;
    private final double fieldLength;
    private final double fieldWidth;

    public BallSimulator(
            BallConstants constants,
            double robotDt,
            double physicsDt,
            double fieldLength,
            double fieldWidth) {

        this.constants = constants;
        this.robotDt = robotDt;
        this.physicsDt = physicsDt;
        this.fieldLength = fieldLength;
        this.fieldWidth = fieldWidth;
    }

    public int addBall(BallState initialState) {
        balls.add(initialState);
        return balls.size() - 1;
    }

    public void update() {
        int steps = (int) Math.ceil(robotDt / physicsDt);
        double dt = robotDt / steps;

        for (int i = 0; i < steps; i++) {
            for (BallState ball : balls) {
                BallPhysics.step(ball, constants, dt);
            }
        }

        // remove balls that leave the field
        for (int i = balls.size() - 1; i >= 0; i--) {
            if (isOutOfBounds(balls.get(i).position)) {
                balls.remove(i);
            }
        }
    }

    @Log
    public Pose3d[] getBallPoses() {
        Pose3d[] poses = new Pose3d[balls.size()];

        for (int i = 0; i < balls.size(); i++) {
            BallState b = balls.get(i);
            poses[i] = new Pose3d(b.position, b.orientation);
        }

        return poses;
    }

    private boolean isOutOfBounds(Translation3d p) {
        return p.getZ() < -OUT_OF_BOUNDS_MARGIN ||
                p.getX() < -OUT_OF_BOUNDS_MARGIN ||
                p.getY() < -OUT_OF_BOUNDS_MARGIN ||
                p.getX() > fieldLength + OUT_OF_BOUNDS_MARGIN ||
                p.getY() > fieldWidth + OUT_OF_BOUNDS_MARGIN;
    }
}