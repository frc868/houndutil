// package com.techhounds.houndutil.houndauto;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import java.lang.reflect.Constructor;
// import java.lang.reflect.InvocationTargetException;
// import java.util.ArrayList;
// import java.util.List;

// /**
//  * Provides utilities for converting blue-side PathPlanner objects to red-side.
//  * Reflection is needed
//  * due to private or protected fields within PathPlanner's API.
//  *
//  * <p>
//  * These transformations assume an absolute field origin on the blue alliance
//  * driver station, on
//  * the scoring table side (away from the human player station).
//  *
//  * <p>
//  * +X is the direction from blue alliance driver station to red alliance driver
//  * station.
//  *
//  * <p>
//  * +Y is the direction from scoring table to human player station.
//  */
// public class TrajectoryReflector {
//     private static Constructor<PathPlannerTrajectory> constructor;

//     static {
//         try {
//             // Access the private constructor that builds a trajectory from states
//             constructor = PathPlannerTrajectory.class.getDeclaredConstructor(
//                     List.class, List.class, StopEvent.class, StopEvent.class, boolean.class);
//             constructor.setAccessible(true);
//         } catch (SecurityException | NoSuchMethodException e) {
//             System.err.println(
//                     "Could not access private fields via reflection in PathPlannerTrajectory.");
//             e.printStackTrace(System.err);
//         }
//     }

//     /**
//      * Transforms a blue-side PathPlannerState to a red-side PathPlannerState. This
//      * should not need to
//      * be called from outside this class.
//      *
//      * @param state The blue-side state of a blue-side trajectory
//      * @return A new red-side state, or the same state if the DriverStation is set
//      *         to blue.
//      */
//     private static PathPlannerState reflectiveTransformState(PathPlannerState state, double fieldLength) {
//         PathPlannerState transformedState = new PathPlannerState();

//         // Move it to the other side of the field, with an absolute origin on blue side
//         // Mirror the X, keep the Y the same.
//         Translation2d transformedTranslation = new Translation2d(
//                 fieldLength - state.poseMeters.getX(), state.poseMeters.getY());

//         // The instantaneous heading of the trajectory needs to be negated
//         Rotation2d transformedHeading = state.poseMeters.getRotation().times(-1);
//         // The holonomic heading needs to be negated and rotated
//         Rotation2d transformedHolonomicRotation = state.holonomicRotation.times(-1).plus(Rotation2d.fromDegrees(180));

//         transformedState.timeSeconds = state.timeSeconds;
//         // Negate the velocity. If traveling from community to mid field on blue, the +X
//         // velocity is
//         // positive. If doing so on red, the +X velocity is negative.
//         transformedState.velocityMetersPerSecond = -state.velocityMetersPerSecond;
//         transformedState.accelerationMetersPerSecondSq = state.accelerationMetersPerSecondSq;
//         transformedState.poseMeters = new Pose2d(transformedTranslation, transformedHeading);
//         transformedState.angularVelocityRadPerSec = -state.angularVelocityRadPerSec;
//         transformedState.holonomicRotation = transformedHolonomicRotation;
//         transformedState.holonomicAngularVelocityRadPerSec = -state.holonomicAngularVelocityRadPerSec;
//         transformedState.curvatureRadPerMeter = -state.curvatureRadPerMeter;

//         return transformedState;
//     }

//     /**
//      * Transforms a blue-side PathPlannerTrajectory to a red-side
//      * PathPlannerTrajectory. In the event
//      * where this fails for any reason, an empty trajectory is returned so as not
//      * cause unpredictable
//      * behavior.
//      *
//      * @param trajectory the blue-side trajectory to transform
//      * @return the equivalent red-side trajectory
//      */
//     public static PathPlannerTrajectory reflectiveTransformTrajectory(
//             PathPlannerTrajectory trajectory, double fieldLength) {
//         List<Trajectory.State> transformedStates = new ArrayList<>();

//         try {
//             // Convert all the trajectory states to red-side
//             for (Trajectory.State s : trajectory.getStates()) {
//                 PathPlannerState state = (PathPlannerState) s;
//                 transformedStates.add(reflectiveTransformState(state, fieldLength));
//             }

//             // Call the now unhidden constructor
//             return constructor.newInstance(
//                     transformedStates,
//                     trajectory.getMarkers(),
//                     trajectory.getStartStopEvent(),
//                     trajectory.getEndStopEvent(),
//                     trajectory.fromGUI);
//         } catch (IllegalArgumentException
//                 | IllegalAccessException
//                 | InstantiationException
//                 | InvocationTargetException e) {
//             e.printStackTrace();
//             // If this fails on the real field, return an empty trajectory instead of
//             // crashing
//             return new PathPlannerTrajectory();
//         }
//     }

//     public static Pose2d reflectiveTransformPose(Pose2d pose, double fieldLength) {
//         Translation2d transformedTranslation = new Translation2d(
//                 fieldLength - pose.getTranslation().getX(), pose.getTranslation().getY());

//         // The instantaneous heading of the trajectory needs to be negated
//         Rotation2d transformedHeading = pose.getRotation().plus(Rotation2d.fromDegrees(180));

//         return new Pose2d(transformedTranslation, transformedHeading);
//     }
// }

// TODO