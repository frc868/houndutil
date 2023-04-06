package com.techhounds.houndutil.houndlib;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class AprilTagPhotonCamera {
    private String name;
    private PhotonCamera photonCamera;
    private PhotonPoseEstimator photonPoseEstimator;
    private Transform3d robotToCam;

    private Pose3d currentDetectedRobotPose = new Pose3d();
    private List<Pose3d> currentDetectedAprilTags = new ArrayList<Pose3d>();
    private boolean hasPose = false;

    public AprilTagPhotonCamera(String name, Transform3d robotToCam) {
        try {
            AprilTagFieldLayout atfl = AprilTagFieldLayout
                    .loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);

            photonCamera = new PhotonCamera(name);
            this.name = name;

            this.robotToCam = robotToCam;
            photonPoseEstimator = new PhotonPoseEstimator(atfl, PoseStrategy.MULTI_TAG_PNP, photonCamera,
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
        result.targets.removeIf((target) -> target.getPoseAmbiguity() > 0.2);
        result.targets.removeIf((target) -> target.getFiducialId() > 8);
        result.targets.removeIf((target) -> target.getBestCameraToTarget().getTranslation().getNorm() > 2);

        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        Optional<EstimatedRobotPose> estimatedRobotPose = photonPoseEstimator.update(result);

        if (estimatedRobotPose.isPresent()) {
            currentDetectedAprilTags = getPosesFromTargets(estimatedRobotPose.get().targetsUsed, prevEstimatedRobotPose,
                    robotToCam);
            currentDetectedRobotPose = estimatedRobotPose.get().estimatedPose;
            hasPose = true;
        } else {
            currentDetectedAprilTags = List.of();
            currentDetectedRobotPose = new Pose3d(-100, -100, 100, new Rotation3d());
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

    public List<Pose3d> getCurrentlyDetectedAprilTags() {
        return currentDetectedAprilTags;
    }

    public Pose3d getCurrentlyDetectedRobotPose() {
        return currentDetectedRobotPose;
    }

    public boolean hasPose() {
        return hasPose;
    }
}
