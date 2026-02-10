package com.techhounds.houndutil.houndlib.subsystems;

import java.util.function.Supplier;

import com.techhounds.houndutil.houndlib.TriConsumer;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Base scaffolding for a "vision" subsystem. Should update the pose estimator
 * with latency-compensated vision measurements and calculated standard
 * deviations based on desired factors.
 */
public interface BaseVision {
    /**
     * Updates the measurement consumer with the latest data from all cameras.
     * Should be called periodically.
     */
    public void updateVisionEstimates();

    /**
     * Gets the supplied camera poses in the global frame, based off of the robot
     * pose.
     * 
     * @return the poses of each registered camera
     */
    public Pose3d[] getCameraPoses();

    /**
     * Gets the poses of all AprilTags in the current field layout.
     * 
     * @return the poses of all AprilTags
     */
    public Pose3d[] getAprilTagPoses();

    /**
     * Sets the pose estimator to use for the vision system.
     * 
     * @param poseEstimator the pose estimator to use
     */
    public void setPoseEstimator(SwerveDrivePoseEstimator poseEstimator);

    /**
     * Sets the consumer for vision measurements, taking in the pose, the timestamp,
     * and the standard deviations of a given measurement.
     * 
     * @param visionMeasurementConsumer the consumer to use
     */
    public void setVisionMeasurementConsumer(TriConsumer<Pose2d, Double, Matrix<N3, N1>> visionMeasurementConsumer);

    /**
     * Sets the consumer for vision measurements, taking in the pose, the timestamp,
     * and the standard deviations of a given measurement.
     * 
     * @param visionMeasurementConsumer the consumer to use
     */
    public void setPreciseVisionMeasurementConsumer(
            TriConsumer<Pose2d, Double, Matrix<N3, N1>> preciseVisionMeasurementConsumer);

    /**
     * Sets the supplier for the ground truth simulation pose.
     * 
     * @param simPoseSupplier the pose supplier to use
     */
    public void setSimPoseSupplier(Supplier<Pose2d> simPoseSupplier);

    /**
     * Sets the supplier for the robot's chassis speeds, used to invalidate pose
     * measurements when the robot is moving too fast.
     * 
     * @param chassisSpeedsSupplier the supplier to use
     */
    public void setChassisSpeedsSupplier(Supplier<ChassisSpeeds> chassisSpeedsSupplier);

    /**
     * Sets the supplier for the robot's heading.
     * 
     * @param chassisSpeedsSupplier the supplier to use
     */
    public void setHeadingSupplier(Supplier<Rotation2d> headingSupplier);

    /**
     * Gets an aggregated list of the latest cached measurements from all cameras,
     * so that they can be displayed easily. Does not actually update the cameras.
     * 
     * @return the latest cached measurements from all cameras
     */
    public Pose3d[] getEstimatedRobotPoses();

    /**
     * Gets an aggregated list of the latest cached measurements from all cameras,
     * so that they can be displayed easily. Does not actually update the cameras.
     * 
     * @return the latest cached measurements from all cameras
     */
    public Pose3d[] getEstimatedPreciseRobotPoses();

    /**
     * Gets an aggregated list of the latest cached measurements from all cameras,
     * so that they can be displayed easily. Does not actually update the cameras.
     * 
     * @return the latest cached measurements from all cameras
     */
    public Pose3d[] getLastUsedRobotPoses();

    /**
     * Gets an aggregated list of the latest cached measurements from all cameras,
     * so that they can be displayed easily. Does not actually update the cameras.
     * 
     * @return the latest cached measurements from all cameras
     */
    public Pose3d[] getLastUsedPreciseRobotPoses();

    /**
     * Gets an aggregated list of all of the detected AprilTags from the latest
     * cached measurements from all cameras, so that they can be displayed easily.
     * Does not actually update the cameras.
     * 
     * @return the latest cached detected AprilTags from all cameras
     */
    public Pose3d[] getDetectedAprilTags();
}
