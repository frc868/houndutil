package com.techhounds.houndutil.houndauto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Utility class for reflecting poses when working with a mirrored field (2023,
 * 2024).
 */
public class Reflector {
    /**
     * Reflects a {@link Pose2d} to the opposite side of a field given a field
     * length. Inverts the heading of the pose (e.g. facing towards one alliance
     * wall will get transformed into facing the other).
     * 
     * @param pose        the pose to reflect
     * @param fieldLength the length of the field, in meters
     * @return the reflected pose
     */
    public static Pose2d reflectPose2d(Pose2d pose, double fieldLength) {
        Translation2d transformedTranslation = new Translation2d(
                fieldLength - pose.getTranslation().getX(), pose.getTranslation().getY());

        Rotation2d transformedHeading = new Rotation2d(Math.PI - pose.getRotation().getRadians());

        return new Pose2d(transformedTranslation, transformedHeading);
    }

    /**
     * Reflects a {@link Pose3d} to the opposite side of a field given a field
     * length. Inverts the heading of the pose (e.g. facing towards one alliance
     * wall will get transformed into facing the other).
     * 
     * @param pose        the pose to reflect
     * @param fieldLength the length of the field, in meters
     * @return the reflected pose
     */
    public static Pose3d reflectPose3d(Pose3d pose, double fieldLength) {
        Translation3d transformedTranslation = new Translation3d(
                fieldLength - pose.getTranslation().getX(), pose.getTranslation().getY(), pose.getZ());

        Rotation3d transformedHeading = pose.getRotation()
                .plus(new Rotation3d(0, 0, Math.PI - pose.getRotation().getZ()));

        return new Pose3d(transformedTranslation, transformedHeading);
    }
}
