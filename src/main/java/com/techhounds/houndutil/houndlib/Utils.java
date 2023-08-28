package com.techhounds.houndutil.houndlib;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Utils {

    public static boolean limitMechanism(boolean isLowerTriggered, boolean isUpperTriggered, double value) {
        return (value > 0.0 && isUpperTriggered) || (value < 0.0 && isLowerTriggered);
    }

    public static PathPoint convertPose2dToPathPoint(Pose2d pose, boolean useRotationAsHolonomic) {
        return useRotationAsHolonomic
                ? new PathPoint(pose.getTranslation(), pose.getRotation(), pose.getRotation())
                : new PathPoint(pose.getTranslation(), pose.getRotation());
    }

    public static PathPoint convertPose2dToPathPoint(Pose2d pose, Rotation2d holonomicRotation) {
        return new PathPoint(pose.getTranslation(), pose.getRotation(), holonomicRotation);
    }

    public static PathPoint convertTranslation2dToPathPoint(Translation2d translation, Rotation2d rotation,
            Rotation2d holonomicRotation) {
        return new PathPoint(translation, rotation, holonomicRotation);
    }
}
