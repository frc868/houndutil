package com.techhounds.houndutil.houndlib;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.techhounds.houndutil.houndlog.interfaces.Log;
import com.techhounds.houndutil.houndlog.interfaces.LoggedObject;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

@LoggedObject
public class AprilTagPhotonCamera {
    private String name;
    private PhotonCamera photonCamera;
    private PhotonPoseEstimator photonPoseEstimator;
    private Transform3d robotToCam;

    @Log(name = "Estimated Robot Pose")
    private Pose3d estimatedRobotPose = new Pose3d();

    private List<Pose3d> detectedAprilTags = new ArrayList<Pose3d>();
    @Log(name = "Detected AprilTags")
    private Supplier<double[]> detectedAprilTagsAS = () -> AdvantageScopeSerializer.serializePose3ds(detectedAprilTags);

    @Log(name = "Has Pose")
    private boolean hasPose = false;

    @Log(name = "Target Count")
    private int targetCount = 0;

    @Log(name = "Target 1 X Transform")
    private double target1X = 0;
    @Log(name = "Target 1 Y Transform")
    private double target1Y = 0;
    @Log(name = "Target 1 Theta Transform")
    private double target1Theta = 0;
    @Log(name = "Target 1 Ambiguity")
    private double target1Amb = 0;
    @Log(name = "Timestamp")
    private double timestamp = 0;

    public AprilTagPhotonCamera(String name, Transform3d robotToCam) {
        try {
            AprilTagFieldLayout atfl = AprilTagFieldLayout
                    .loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);

            photonCamera = new PhotonCamera(name);
            this.name = name;

            this.robotToCam = robotToCam;
            photonPoseEstimator = new PhotonPoseEstimator(atfl, PoseStrategy.LOWEST_AMBIGUITY, photonCamera,
                    robotToCam);
        } catch (IOException e) {
            e.printStackTrace();
        }

    }

    /**
     * @param estimatedRobotPose The current best guess at robot pose
     * @return A pair of the fused camera observations to a single Pose2d on the
     *         field, and the time of the observation.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(
            Pose2d prevEstimatedRobotPose) {
        PhotonPipelineResult result = photonCamera.getLatestResult();
        // result.targets.removeIf((target) -> target.getPoseAmbiguity() > 0.2);
        // result.targets.removeIf((target) -> target.getFiducialId() > 8);
        // result.targets.removeIf((target) ->
        // target.getBestCameraToTarget().getTranslation().getNorm() > 4);
        targetCount = result.targets.size();

        // photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        Optional<EstimatedRobotPose> estimatedRobotPose = photonPoseEstimator.update(result);

        if (result.targets.size() > 0) {
            Transform3d bestTransform = result.targets.get(0).getBestCameraToTarget();
            target1X = bestTransform.getX();
            target1Y = bestTransform.getY();
            target1Theta = bestTransform.getRotation().getAngle();
            target1Amb = result.targets.get(0).getPoseAmbiguity();

            timestamp = result.getTimestampSeconds();
        }
        if (estimatedRobotPose.isPresent()) {
            this.estimatedRobotPose = estimatedRobotPose.get().estimatedPose;
            detectedAprilTags = getPosesFromTargets(result.targets, this.estimatedRobotPose.toPose2d(), robotToCam);
            hasPose = true;
        } else {
            hasPose = false;
        }

        return estimatedRobotPose;
    }

    private List<Pose3d> getPosesFromTargets(List<PhotonTrackedTarget> targets, Pose2d robotPose,
            Transform3d robotToCam) {
        List<Pose3d> poses = new ArrayList<Pose3d>();
        for (int i = 0; i < targets.size(); i++) {
            Pose3d robotPose3d = new Pose3d(robotPose);
            Transform3d camToTarget = targets.get(i).getBestCameraToTarget();
            poses.add(robotPose3d.plus(robotToCam).plus(camToTarget));
        }
        return poses;
    }

    public String getName() {
        return name;
    }

    // public Optional<Pose2d> getAprilTagPoseRelativeToRobot(Pose2d robotPose) {
    // PhotonPipelineResult result = photonCamera.getLatestResult();
    // if (result.getBestTarget() != null) {
    // Transform3d camToTarget = result.getBestTarget().getBestCameraToTarget();
    // Pose2d pose = new Pose3d(robotPose).plus(camToTarget).toPose2d();

    // return Optional.of(
    // new Pose2d(
    // xFilter.calculate(pose.getX()),
    // yFilter.calculate(pose.getY()),
    // new Rotation2d(thetaFilter.calculate(pose.getRotation().getRadians()))));
    // } else {
    // return Optional.empty();
    // }
    // }
}
