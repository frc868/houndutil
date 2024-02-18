package com.techhounds.houndutil.houndauto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class Reflector {
    public static Pose2d reflectPose2d(Pose2d pose, double fieldLength) {
        Translation2d transformedTranslation = new Translation2d(
                fieldLength - pose.getTranslation().getX(), pose.getTranslation().getY());

        Rotation2d transformedHeading = pose.getRotation().plus(Rotation2d.fromDegrees(180));

        return new Pose2d(transformedTranslation, transformedHeading);
    }

    public static Pose3d reflectPose3d(Pose3d pose, double fieldLength) {
        Translation3d transformedTranslation = new Translation3d(
                fieldLength - pose.getTranslation().getX(), pose.getTranslation().getY(), pose.getZ());

        Rotation3d transformedHeading = pose.getRotation().plus(new Rotation3d(0, 0, Math.PI));

        return new Pose3d(transformedTranslation, transformedHeading);
    }
}
