package com.techhounds.houndutil.houndauto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Reflector {
    public static Pose2d reflectiveTransformPose(Pose2d pose, double fieldLength) {
        Translation2d transformedTranslation = new Translation2d(
                fieldLength - pose.getTranslation().getX(), pose.getTranslation().getY());

        Rotation2d transformedHeading = pose.getRotation().plus(Rotation2d.fromDegrees(180));

        return new Pose2d(transformedTranslation, transformedHeading);
    }
}
