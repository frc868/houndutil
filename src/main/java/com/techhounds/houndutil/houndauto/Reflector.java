package com.techhounds.houndutil.houndauto;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
/**
 * Utility class for reflecting poses when working with a mirrored field (2023,
 * 2024).
 */
public class Reflector {
        public class Constants {
                //Temporary place for constants
                /** The length (x-axis) of the field */
                public static final Distance FIELD_LENGTH = Inches.of(651.22);
                /** The width (y-axis) of the field */
                public static final Distance FIELD_WIDTH = Inches.of(317.69);
        }

        /**
         * Reflects a {@link Pose2d} to the opposite side of the driver station (Left to
         * Right) Inverts heading to make it face the same way field relative
         * 
         * @param pose       the pose to reflect
         * @return the reflected pose
         */
        public static Pose2d reflectWidthPose2d(Pose2d pose) {
                Translation2d transformedTranslation = new Translation2d(
                                pose.getTranslation().getX(),
                                Constants.FIELD_WIDTH.in(Meters) - pose.getTranslation().getY());
                Rotation2d transformedHeading = new Rotation2d(
                                -pose.getRotation().getRadians());

                return new Pose2d(transformedTranslation, transformedHeading);
        }

        /**
         * Reflects a {@link Pose2d} to the opposite side of a field given a field
         * length. Inverts the heading of the pose (e.g. facing towards one alliance
         * wall will get transformed into facing the other).
         * 
         * @param pose        the pose to reflect
         * @return the reflected pose
         */
        public static Pose2d reflectPose2d(Pose2d pose) {
                Translation2d transformedTranslation = new Translation2d(
                                Constants.FIELD_LENGTH.in(Meters) - pose.getTranslation().getX(), pose.getTranslation().getY());

                Rotation2d transformedHeading = new Rotation2d(Math.PI - pose.getRotation().getRadians());

                return new Pose2d(transformedTranslation, transformedHeading);
        }

        public static Pose2d rotatePoseAcrossField(Pose2d pose) {
                
                double halfFieldX = Constants.FIELD_LENGTH.in(Meters) / 2.0;
                double halfFieldY = Constants.FIELD_WIDTH.in(Meters) / 2.0;

                // Reflect around the field center
                Translation2d rotatedTranslation = new Translation2d(
                                2 * halfFieldX - pose.getTranslation().getX(),
                                2 * halfFieldY - pose.getTranslation().getY());

                Rotation2d rotatedHeading = pose.getRotation().rotateBy(Rotation2d.fromDegrees(180));

                return new Pose2d(rotatedTranslation, rotatedHeading);

        }

        /**
         * Reflects a {@link Pose3d} to the opposite side of a field given a field
         * length. Inverts the heading of the pose (e.g. facing towards one alliance
         * wall will get transformed into facing the other).
         * 
         * @param pose        the pose to reflect
         * @return the reflected pose
         */
        public static Pose3d reflectPose3d(Pose3d pose) {
                Translation3d transformedTranslation = new Translation3d(
                                Constants.FIELD_LENGTH.in(Meters) - pose.getTranslation().getX(), pose.getTranslation().getY(), pose.getZ());

                Rotation3d transformedHeading = pose.getRotation()
                                .plus(new Rotation3d(0, 0, Math.PI - pose.getRotation().getZ()));

                return new Pose3d(transformedTranslation, transformedHeading);
        }
}
