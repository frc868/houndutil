package com.techhounds.houndutil.houndauto;

import static com.techhounds.houndutil.HoundConstants.*;

import com.techhounds.houndutil.houndlib.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/** Utility class for reflecting and rotating poses when across a field. */
public class Reflector {
    /**
     * Reflects a {@link Pose2d} to the opposite side of the driver station (Left to
     * Right). Inverts heading to make it face the same way field relative.
     * 
     * @param pose the pose to reflect
     * @return the reflected pose
     */
    public static Pose2d reflectWidthPose2d(Pose2d pose) {
        Translation2d transformedTranslation = new Translation2d(pose.getTranslation().getMeasureX(),
                FIELD_WIDTH.minus(pose.getTranslation().getMeasureY()));
        Rotation2d transformedHeading = pose.getRotation().unaryMinus();

        return new Pose2d(transformedTranslation, transformedHeading);
    }

    /**
     * Reflects a {@link Pose2d} to the opposite side of a field. Inverts the
     * heading of the pose (e.g. facing towards one alliance wall will get
     * transformed into facing the other).
     * 
     * @param pose the pose to reflect
     * @return the reflected pose
     */
    public static Pose2d reflectLengthPose2d(Pose2d pose) {
        Translation2d transformedTranslation = new Translation2d(
                FIELD_LENGTH.minus(pose.getTranslation().getMeasureX()), pose.getTranslation().getMeasureY());

        Rotation2d transformedHeading = Rotation2d.kPi.minus(pose.getRotation());

        return new Pose2d(transformedTranslation, transformedHeading);
    }

    /**
     * Rotates a {@link Pose2d} around the field center.
     * 
     * @param pose the pose to rotate
     * @return the rotated pose
     */
    public static Pose2d rotatePoseAcrossField(Pose2d pose) {
        return reflectLengthPose2d(reflectWidthPose2d(pose));
    }

    /**
     * Rotates a given pose to red alliance only if this is the alliance of the
     * robot
     * 
     * @param original the pose to rotate if necessary
     * @return if on blue, the original pose, otherwise the rotated pose
     */
    public static Pose2d rotateBluePoseAcrossFieldIfNecessary(Pose2d original) {
        return Utils.isRobotRedAlliance() ? rotatePoseAcrossField(original) : original;
    }

    /**
     * Reflects a {@link Pose3d} to the opposite side of a field. Inverts the
     * heading of the pose (e.g. facing towards one alliance wall will get
     * transformed into facing the other).
     * 
     * @param pose the pose to reflect
     * @return the reflected pose
     */
    public static Pose3d reflectPose3d(Pose3d pose) {
        Translation3d transformedTranslation = new Translation3d(
                FIELD_LENGTH.minus(pose.getTranslation().getMeasureX()), pose.getTranslation().getMeasureY(),
                pose.getMeasureZ());

        Rotation3d transformedHeading = new Rotation3d(Rotation2d.kPi.minus(pose.getRotation().toRotation2d()));

        return new Pose3d(transformedTranslation, transformedHeading);
    }
}
