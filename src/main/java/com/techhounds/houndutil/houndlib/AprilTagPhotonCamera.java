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

import com.techhounds.houndutil.houndlog.interfaces.Log;
import com.techhounds.houndutil.houndlog.interfaces.LoggedObject;

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

@LoggedObject
public class AprilTagPhotonCamera {
    public static class PhotonCameraConstants {
        public int WIDTH;
        public int HEIGHT;
        public double FOV;
        public double FPS;
        public double AVG_LATENCY;
        public double STDDEV_LATENCY;
    }

    private String name;
    private PhotonCamera photonCamera;
    private PhotonCameraSim cameraSim;
    private PhotonPoseEstimator photonPoseEstimator;
    private Transform3d robotToCam;

    @Log
    private Pose3d estimatedRobotPose = new Pose3d();
    @Log
    private Pose3d[] detectedAprilTags = new Pose3d[0];
    @Log
    private boolean hasPose = false;
    @Log
    private int targetCount = 0;

    private double lastTimestamp = 0;

    public AprilTagPhotonCamera(String name, Transform3d robotToCam, PhotonCameraConstants constants,
            double avgErrorPx, double stdDevErrorPx) {
        this.name = name;
        this.robotToCam = robotToCam;

        photonCamera = new PhotonCamera(name);
        photonPoseEstimator = new PhotonPoseEstimator(AprilTagFields.kDefaultField.loadAprilTagLayoutField(),
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCamera, robotToCam);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

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
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(
            Pose2d prevEstimatedRobotPose) {
        PhotonPipelineResult result = photonCamera.getLatestResult();
        double timestamp = result.getTimestampSeconds();
        boolean newResult = Math.abs(timestamp - lastTimestamp) > 1e-5;
        targetCount = result.targets.size();

        Optional<EstimatedRobotPose> photonEstimatedRobotPose = photonPoseEstimator.update();

        if (newResult) {
            if (photonEstimatedRobotPose.isPresent()) {
                estimatedRobotPose = photonEstimatedRobotPose.get().estimatedPose;
                detectedAprilTags = getPosesFromTargets(result.targets, estimatedRobotPose,
                        robotToCam);
            } else {
                detectedAprilTags = new Pose3d[0];
                estimatedRobotPose = new Pose3d(-100, -100, -100, new Rotation3d());
            }
        }

        hasPose = photonEstimatedRobotPose.isPresent();
        if (newResult)
            lastTimestamp = timestamp;
        return photonEstimatedRobotPose;

    }

    /**
     * The standard deviations of the estimated pose from
     * {@link #getEstimatedGlobalPose()}, for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
     * SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose, Matrix<N3, N1> singleTagStdDevs,
            Matrix<N3, N1> multiTagStdDevs) {
        var estStdDevs = singleTagStdDevs;
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
        if (numTags == 0)
            return estStdDevs;
        avgDist /= numTags;

        if (avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else {
            if (numTags > 1)
                estStdDevs = multiTagStdDevs.times(1 + (avgDist * avgDist / 5));
            else
                estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 5));
        }

        return estStdDevs;
    }

    private Pose3d[] getPosesFromTargets(List<PhotonTrackedTarget> targets, Pose3d estimatedRobotPose,
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

    public String getName() {
        return name;
    }

    public boolean hasPose() {
        return hasPose;
    }

    public PhotonCameraSim getSim() {
        return cameraSim;
    }

    public Transform3d getRobotToCam() {
        return robotToCam;
    }
}
