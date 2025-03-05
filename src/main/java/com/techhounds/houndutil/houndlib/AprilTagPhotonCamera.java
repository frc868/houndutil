package com.techhounds.houndutil.houndlib;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.techhounds.houndutil.houndlog.FaultLogger;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * A camera that uses PhotonVision to derive an estimated pose based on visible
 * AprilTags, using multi-tag PNP on coprocessor as the primary algorithm. Uses
 * the lowest ambiguity measurement for single tags.
 * 
 * Note: you must enable multi-tag PNP on the PhotonVision web UI for each
 * camera for the correct pose strategy to activate.
 */
@LoggedObject
public class AprilTagPhotonCamera {
    /**
     * Common constants for a camera intended to view AprilTags. Should be able to
     * use across multiple cameras. To initialize, create a `static` block in your
     * Constants class and set the values.
     */
    public static class PhotonCameraConstants {
        /** The width of the image, in pixels. */
        public int WIDTH;
        /** The height of the image, in pixels. */
        public int HEIGHT;
        /** The diagonal field of view of the camera, in degrees. */
        public double FOV;
        /** The expected FPS of the camera. Used for simulation. */
        public double FPS;
        /**
         * The expected average latency, in ms, of the camera. Used for simulation. If
         * unsure, use 30ms.
         */
        public double AVG_LATENCY;
        /**
         * The expected standard deviation of the latency, in ms, of the camera. Used
         * for simulation. If unsure, use 15ms.
         */
        public double STDDEV_LATENCY;
    }

    protected String name;
    protected PhotonCamera photonCamera;
    protected PhotonCameraSim cameraSim;
    protected PhotonPoseEstimator photonPoseEstimator;
    protected PhotonPoseEstimator trigSolvePoseEstimator;
    protected Transform3d robotToCam;

    @Log
    protected Pose3d estimatedRobotPose = new Pose3d();
    @Log
    protected Pose3d estimatedTrigPose = new Pose3d();
    @Log
    protected Pose3d[] detectedAprilTags = new Pose3d[0];
    @Log
    protected Pose3d[] detectedTrigAprilTags = new Pose3d[0];
    @Log
    protected boolean hasPose = false;
    protected boolean hasTrigPose = false;
    @Log
    protected int targetCount = 0;

    protected double lastTimestamp = 0;
    protected double lastTrigTimestamp = 0;

    /**
     * Initializes the PhotonVision camera.
     * 
     * @param name          the name of the camera assigned in PhotonVision.
     * @param robotToCam    the transform from the center of the robot (at a
     *                      z-height of 0) to the sensor of the camera. this should
     *                      be as accurate as possible to minimize compounding
     *                      tolerances.
     * @param constants     the common constants for the camera.
     * @param avgErrorPx    the average error of the camera calibration, used for
     *                      simulation. if unsure, use 0.2px.
     * @param stdDevErrorPx the standard deviation of the error of the camera
     *                      calibration, used for simulation. if unsure, use 0.1px.
     */
    public AprilTagPhotonCamera(String name, Transform3d robotToCam, PhotonCameraConstants constants,
            double avgErrorPx, double stdDevErrorPx) {
        this.name = name;
        this.robotToCam = robotToCam;

        photonCamera = new PhotonCamera(name);

        AprilTagFieldLayout reefscapeLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

        ArrayList<AprilTag> reefTags = new ArrayList<>(reefscapeLayout.getTags());
        reefTags.removeIf((a) -> !((a.ID >= 6 && a.ID <= 11) || (a.ID >= 17 && a.ID <= 22)));
        AprilTagFieldLayout reefOnlyLayout = new AprilTagFieldLayout(reefTags, reefscapeLayout.getFieldLength(),
                reefscapeLayout.getFieldWidth());

        photonPoseEstimator = new PhotonPoseEstimator(reefscapeLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);

        trigSolvePoseEstimator = new PhotonPoseEstimator(reefOnlyLayout, PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
                robotToCam);
        // trigSolvePoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        if (RobotBase.isSimulation()) {
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(constants.WIDTH, constants.HEIGHT,
                    Rotation2d.fromDegrees(constants.FOV));
            cameraProp.setCalibError(avgErrorPx, stdDevErrorPx);
            cameraProp.setFPS(constants.FPS);
            cameraProp.setAvgLatencyMs(constants.AVG_LATENCY);
            cameraProp.setLatencyStdDevMs(constants.STDDEV_LATENCY);
            cameraSim = new PhotonCameraSim(photonCamera, cameraProp);

            cameraSim.enableDrawWireframe(true);
        }
        FaultLogger.register(photonCamera);
    }

    /**
     * Gets an estimated pose of the robot on the field, if possible.
     * 
     * @param prevEstimatedRobotPose the previous global pose generated by the
     *                               overall pose estimator. used for legacy
     *                               reasons, in case single-tag algorithms that
     *                               rank based on distance to the previous pose are
     *                               desired.
     * @return if possible, an {@link EstimatedRobotPose}, with the estimated pose,
     *         timestamp, and targets used.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(
            Pose2d prevEstimatedRobotPose) {
        PhotonPipelineResult result = photonCamera.getLatestResult();
        double timestamp = result.getTimestampSeconds();
        boolean newResult = Math.abs(timestamp - lastTimestamp) > 1e-5;

        PhotonPipelineResult pipelineResult = photonCamera.getLatestResult();
        Optional<EstimatedRobotPose> photonEstimatedRobotPose = photonPoseEstimator.update(pipelineResult);

        if (newResult) {
            if (photonEstimatedRobotPose.isPresent()) {
                estimatedRobotPose = photonEstimatedRobotPose.get().estimatedPose;

                detectedAprilTags = getPosesFromTargets(result.targets, estimatedRobotPose,
                        robotToCam);

                // reject the pose if we are over 1m off the ground, or over 1m under the
                // ground.

                if (estimatedRobotPose.getZ() > 1 || estimatedRobotPose.getZ() < -1) {
                    hasPose = photonEstimatedRobotPose.isPresent();
                    return Optional.empty();
                }
            } else {
                detectedAprilTags = new Pose3d[0];
                estimatedRobotPose = new Pose3d(-100, -100, -100, new Rotation3d());
            }
            hasPose = photonEstimatedRobotPose.isPresent();
            lastTimestamp = timestamp;
        }
        return photonEstimatedRobotPose;
    }

    /**
     * Gets an estimated pose of the robot on the field using the trigonometric pose
     * algorithm.
     * 
     * @param prevEstimatedRobotPose the previous global pose generated by the
     *                               overall pose estimator. used for legacy
     *                               reasons, in case single-tag algorithms that
     *                               rank based on distance to the previous pose are
     *                               desired.
     * @return if possible, an {@link EstimatedRobotPose}, with the estimated pose,
     *         timestamp, and targets used.
     */
    public Optional<EstimatedRobotPose> getEstimatedTrigPose() {
        PhotonPipelineResult result = photonCamera.getLatestResult();
        double timestamp = result.getTimestampSeconds();
        boolean newResult = Math.abs(timestamp - lastTrigTimestamp) > 1e-5;
        targetCount = result.targets.size();

        PhotonPipelineResult pipelineResult = photonCamera.getLatestResult();

        Optional<EstimatedRobotPose> photonEstimatedRobotPose = trigSolvePoseEstimator.update(pipelineResult);

        if (newResult) {
            if (photonEstimatedRobotPose.isPresent()) {
                estimatedTrigPose = photonEstimatedRobotPose.get().estimatedPose;
                detectedTrigAprilTags = getPosesFromTargets(result.targets, estimatedTrigPose,
                        robotToCam);

                var tagPose = photonPoseEstimator.getFieldTags()
                        .getTagPose(photonEstimatedRobotPose.get().targetsUsed.get(0).getFiducialId());

                // reject if we are >3m away from the reef
                if (tagPose.isPresent()) {
                    double dist = tagPose.get().toPose2d().getTranslation()
                            .getDistance(estimatedTrigPose.getTranslation().toTranslation2d());
                    if (dist > 3) {
                        estimatedTrigPose = new Pose3d(-100, -100, -100, new Rotation3d());
                        return Optional.empty();
                    }
                }

                // reject the pose if we are over 1m off the ground, or over 1m under the
                // ground.
                if (estimatedTrigPose.getZ() > 1 || estimatedTrigPose.getZ() < -1) {
                    estimatedTrigPose = new Pose3d(-100, -100, -100, new Rotation3d());
                    return Optional.empty();
                }
            } else {
                detectedTrigAprilTags = new Pose3d[0];
                estimatedTrigPose = new Pose3d(-100, -100, -100, new Rotation3d());
            }
            hasPose = photonEstimatedRobotPose.isPresent();
            lastTrigTimestamp = timestamp;
        }
        return photonEstimatedRobotPose;
    }

    public void addHeadingData(double timestampSeconds, Rotation2d heading) {
        photonPoseEstimator.addHeadingData(timestampSeconds, heading);
        trigSolvePoseEstimator.addHeadingData(timestampSeconds, heading);
    }

    /**
     * Returns standard deviations of an estimated pose, based on an equation that
     * accounts for distance from the tags (when far away from tags, the standard
     * deviations are increased to account for increased potential error).
     * 
     * If the average distance away from the tags is greater than six meters, the
     * measurement is ignored.
     *
     * @param estimatedPose the estimated pose to guess standard deviations for
     */
    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose, Matrix<N3, N1> singleTagStdDevs,
            Matrix<N3, N1> multiTagStdDevs) {
        var targets = photonCamera.getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = photonPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty())
                continue;
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }

        avgDist /= numTags;

        // if the average distance is > 6 meters, use maximum value standard deviations,
        // effectively nullifying the measurement
        if (avgDist > 6)
            return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else {
            if (numTags > 1)
                // multiplies the standard deviations by 1 + avgDist^2/5. this was found by
                // others in the community to be an effective way to scale standard deviations
                // by distance.
                return multiTagStdDevs.times(1 + (avgDist * avgDist / 5));
            else
                return singleTagStdDevs.times(1 + (avgDist * avgDist / 5));
        }
    }

    /**
     * Gets the poses of the detected AprilTags given a measurement. Used for
     * diagnostics.
     * 
     * @param targets            the targets used for the measurement
     * @param estimatedRobotPose the estimated robot pose created from the
     *                           measurement
     * @param robotToCam         the transform from the center of the robot to the
     *                           sensor of the camera.
     * @return an array of {@link Pose3d}s with the positions of where the camera
     *         believes the AprilTags it detected are located.
     */
    protected Pose3d[] getPosesFromTargets(List<PhotonTrackedTarget> targets, Pose3d estimatedRobotPose,
            Transform3d robotToCam) {
        List<Pose3d> poses = new ArrayList<Pose3d>();
        for (int i = 0; i < targets.size(); i++) {
            Transform3d camToTarget = targets.get(i).getBestCameraToTarget();
            poses.add(estimatedRobotPose.plus(robotToCam).plus(camToTarget));
        }

        Pose3d[] poseArray = new Pose3d[poses.size()];
        poses.toArray(poseArray);
        return poseArray;
    }

    /**
     * Gets the transform3d robot relative of detected apriltag of tagId. Used for
     * precision
     * alignment.
     * 
     * @param tagId the id of the apriltag to get the relative transform of
     * @return an array of Transform3d of where the camera believes the apriltags
     *         are located relative to the vehicle.
     */
    public Transform3d getRelativeTransformsOfTarget(int tagId) {

        PhotonPipelineResult result = photonCamera.getLatestResult();
        double timestamp = result.getTimestampSeconds();
        boolean newResult = Math.abs(timestamp - lastTimestamp) > 1e-5;
        targetCount = result.targets.size();

        PhotonPipelineResult pipelineResult = photonCamera.getLatestResult();

        if (newResult) {
            for (int i = 0; i < pipelineResult.targets.size(); i++) {
                if (pipelineResult.targets.get(i).getFiducialId() == tagId) {
                    Transform3d camToTarget = pipelineResult.targets.get(i).getBestCameraToTarget();
                    return camToTarget;
                }
            }
        }
        return null;
    }

    /**
     * Gets the name of the camera.
     * 
     * @return the name of the camera
     */
    public String getName() {
        return name;
    }

    /**
     * Gets whether the camera is currently producing a pose measurement.
     * 
     * @return if the camera is currently producing a pose measurement
     */
    public boolean hasPose() {
        return hasPose;
    }

    /**
     * Gets the simulation object of the camera, to add to a global vision
     * simulator.
     * 
     * @return the underlying {@link PhotonCameraSim} representing this camera
     */
    public PhotonCameraSim getSim() {
        return cameraSim;
    }

    /**
     * Gets the provided transformation from the center of the robot to the sensor
     * of the camera.
     * 
     * @return the transformation from the center of the robot to the sensor of the
     *         camera
     */
    public Transform3d getRobotToCam() {
        return robotToCam;
    }

    /**
     * Gets the last estimated robot pose. Used for creating a log object containing
     * estimated robot poses from all valid cameras at once.
     * 
     * This method does <b>not</b> call
     * {@link AprilTagPhotonCamera#getEstimatedGlobalPose()}.
     * 
     * @return the last estimated robot pose.
     */
    public Pose3d getLoggedEstimatedRobotPose() {
        return estimatedRobotPose;
    }

    /**
     * Gets the last estimated precise robot pose. Used for creating a log object
     * containing
     * estimated robot poses from all valid cameras at once.
     * 
     * This method does <b>not</b> call
     * {@link AprilTagPhotonCamera#getEstimatedTrigPose()}.
     * 
     * @return the last estimated robot pose.
     */
    public Pose3d getLoggedEstimatedTrigRobotPose() {
        return estimatedTrigPose;
    }

    /**
     * Gets the last estimated AprilTag positions. Used for creating a log object
     * containing estimated AprilTag positions from all valid cameras at once.
     * 
     * This method does <b>not</b> call
     * {@link AprilTagPhotonCamera#getEstimatedGlobalPose()}.
     * 
     * @return the last estimated AprilTag positions.
     */
    public Pose3d[] getLoggedDetectedAprilTags() {
        return detectedAprilTags;
    }

    /**
     * Gets the last estimated AprilTag positions. Used for creating a log object
     * containing estimated AprilTag positions from all valid cameras at once.
     * 
     * This method does <b>not</b> call
     * {@link AprilTagPhotonCamera#getEstimatedGlobalPose()}.
     * 
     * @return the last estimated AprilTag positions.
     */
    public Pose3d[] getLoggedDetectedTrigAprilTags() {
        return detectedTrigAprilTags;
    }
}
