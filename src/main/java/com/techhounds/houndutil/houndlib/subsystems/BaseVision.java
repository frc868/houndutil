package com.techhounds.houndutil.houndlib.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

/**
 * Base scaffolding for a "vision" subsystem. Should update the pose estimator
 * with latency-compensated vision measurements and calculated standard
 * deviations based on desired factors.
 */
public interface BaseVision {
    /**
     * Updates the pose estimator with vision measurements.
     */
    public void updatePoseEstimator();

    /**
     * Gets the positions of the cameras relative to the center of the robot.
     * 
     * @return an array of camera poses
     */
    public Pose3d[] getCameraPoses();

    /**
     * Gets the positions of all of the known AprilTags on the field.
     * 
     * @return an array of AprilTag poses
     */
    public Pose3d[] getAprilTagPoses();

    /**
     * Sets the outside pose estimator object to use internally.
     * 
     * @param poseEstimator the pose estimator object to apply vision measurements
     *                      to
     */
    public void setPoseEstimator(SwerveDrivePoseEstimator poseEstimator);

    /**
     * Sets the outside supplier of the ground-truth simulation pose, used for the
     * PhotonVision VisionSystemSim.
     * 
     * @param simPoseSupplier a Supplier of Pose2ds containing the ground-truth
     *                        simulation pose
     */
    public void setSimPoseSupplier(Supplier<Pose2d> simPoseSupplier);
}
