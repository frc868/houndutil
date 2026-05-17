package com.techhounds.houndutil.houndlib.vision;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.VisionSystemSim;

import com.techhounds.houndutil.houndlib.TriConsumer;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

/**
 * A group of {@link AprilTagPhotonCamera}. Only works for swerve drive robots.
 */
@LoggedObject
public class AprilTagPhotonCameraGroup {
    /** The layout of AprilTags on the field */
    private final AprilTagFieldLayout tagLayout;

    /** Supplier for drivetrain heading, used to provide rotation information. */
    private Supplier<Rotation2d> headingSupplier;

    /**
     * Supplier for drivetrain chassis speeds, used to disable pose estimation when
     * moving too quickly.
     */
    private Supplier<ChassisSpeeds> chassisSpeedsSupplier;

    /**
     * Pose supplier for a ground source of truth pose via odometry, for simulation.
     */
    private Supplier<Pose2d> simPoseSupplier;

    /**
     * The pose estimator to receive the latest robot position from (used for
     * logging and the "closest to last pose" strategy).
     */
    private SwerveDrivePoseEstimator poseEstimator;

    /**
     * The consumer for vision measurements, taking in the pose, the timestamp, and
     * the standard deviations.
     */
    private TriConsumer<Pose2d, Double, Matrix<N3, N1>> visionMeasurementConsumer;

    /**
     * The consumer for trig vision measurements, taking in the pose, the timestamp,
     * and the standard deviations.
     */
    private TriConsumer<Pose2d, Double, Matrix<N3, N1>> preciseVisionMeasurementConsumer;

    /**
     * The standard deviations (x, y, theta) for vision measurements with only one
     * AprilTag.
     */
    private final Matrix<N3, N1> singleTagStdDevs;
    /**
     * The standard deviations (x, y, theta) for vision measurements with multiple
     * AprilTags.
     */
    private final Matrix<N3, N1> multiTagStdDevs;
    /**
     * The standard deviations (x, y, theta) for vision measurements with multiple
     * AprilTags during teleop.
     */
    private final Matrix<N3, N1> multiTagTeleopStdDevs;
    /**
     * The standard deviations (x, y, theta) for trig vision measurements with only
     * one AprilTag.
     */
    private final Matrix<N3, N1> singleTagPreciseStdDevs;

    /**
     * The maximum velocity the robot can be travelling and still feed new poses
     * into the {@code visionMeasurementConsumer}.
     */
    private final LinearVelocity maxPoseEstimationChassisSpeed;
    /**
     * The maximum velocity the robot can be travelling and still feed new poses
     * into the {@code preciseVisionMeasurementConsumer}.
     */
    private final LinearVelocity maxPrecisePoseEstimationChassisSpeed;

    /** An array of all the cameras. */
    private final AprilTagPhotonCamera[] cameras;

    /** A simulation of this vision subsystem. */
    private final VisionSystemSim sim = new VisionSystemSim("main");

    /** The latest poses. */
    private final Pose3d[] latestUsedPoses;
    /** The latest trig poses. */
    private final Pose3d[] latestUsedTrigPoses;

    /**
     * Initializes a group of {@link AprilTagPhotonCamera}.
     * 
     * @param tagLayout                            the AprilTag layout of the field
     * @param headingSupplier                      a supplier of the robot chassis'
     *                                             heading
     * @param chassisSpeedsSupplier                a supplier of the robot chassis'
     *                                             speeds
     * @param simPoseSupplier                      a supplier of the robot's sim
     *                                             pose
     * @param poseEstimator                        the robot's pose estimator
     * @param visionMeasurementConsumer            a consumer of vision
     *                                             measurements, see:
     *                                             {@code KrakenSwerveDrive.addVisionMeasurement(...)}
     * @param preciseVisionMeasurementConsumer     a consumer of trig vision
     *                                             measurements, see:
     *                                             {@code KrakenSwerveDrive.addPreciseVisionMeasurement(...)}
     * @param singleTagStdDevs                     the standard deviations for
     *                                             single-tag estimates
     * @param multiTagStdDevs                      the standard deviations for
     *                                             multi-tag estimates
     * @param multiTagTeleopStdDevs                the standard deviations for
     *                                             multi-tag estimates in teleop
     * @param singleTagPreciseStdDevs              the standard deviations for
     *                                             single-tag trig estimates
     * @param maxPoseEstimationChassisSpeed        the max speed to be moving but
     *                                             still getting accurate poses
     * @param maxPrecisePoseEstimationChassisSpeed the max speed to be moving but
     *                                             still getting accurate trig poses
     * @param cameras                              the {@link AprilTagPhotonCamera}s
     *                                             in this group
     */
    public AprilTagPhotonCameraGroup(
            AprilTagFieldLayout tagLayout,
            Supplier<Rotation2d> headingSupplier,
            Supplier<ChassisSpeeds> chassisSpeedsSupplier,
            Supplier<Pose2d> simPoseSupplier,
            SwerveDrivePoseEstimator poseEstimator,
            TriConsumer<Pose2d, Double, Matrix<N3, N1>> visionMeasurementConsumer,
            TriConsumer<Pose2d, Double, Matrix<N3, N1>> preciseVisionMeasurementConsumer,
            Matrix<N3, N1> singleTagStdDevs,
            Matrix<N3, N1> multiTagStdDevs,
            Matrix<N3, N1> multiTagTeleopStdDevs,
            Matrix<N3, N1> singleTagPreciseStdDevs,
            LinearVelocity maxPoseEstimationChassisSpeed,
            LinearVelocity maxPrecisePoseEstimationChassisSpeed,
            AprilTagPhotonCamera... cameras) {
        this.tagLayout = tagLayout;

        this.headingSupplier = Objects.requireNonNull(headingSupplier, "headingSupplier is null");
        this.chassisSpeedsSupplier = Objects.requireNonNull(chassisSpeedsSupplier, "chassisSpeedsSupplier is null");
        this.simPoseSupplier = Objects.requireNonNull(simPoseSupplier, "simPoseSupplier is null");
        this.poseEstimator = Objects.requireNonNull(poseEstimator, "poseEstimator is null");
        this.visionMeasurementConsumer = Objects.requireNonNull(visionMeasurementConsumer,
                "visionMeasurementConsumer is null");
        this.preciseVisionMeasurementConsumer = Objects.requireNonNull(preciseVisionMeasurementConsumer,
                "preciseVisionMeasurementConsumer is null");

        this.singleTagStdDevs = singleTagStdDevs;
        this.multiTagStdDevs = multiTagStdDevs;
        this.multiTagTeleopStdDevs = multiTagTeleopStdDevs;
        this.singleTagPreciseStdDevs = singleTagPreciseStdDevs;

        this.maxPoseEstimationChassisSpeed = maxPoseEstimationChassisSpeed;
        this.maxPrecisePoseEstimationChassisSpeed = maxPrecisePoseEstimationChassisSpeed;

        this.cameras = cameras;
        latestUsedPoses = new Pose3d[cameras.length];
        latestUsedTrigPoses = new Pose3d[cameras.length];

        for (int i = 0; i < cameras.length; i++) {
            latestUsedPoses[i] = Pose3d.kZero;
            latestUsedTrigPoses[i] = Pose3d.kZero;
        }

        if (RobotBase.isSimulation()) {
            sim.addAprilTags(tagLayout);
            for (AprilTagPhotonCamera camera : cameras) {
                sim.addCamera(camera.getSim(), camera.getRobotToCam());
            }
        }
    }

    /**
     * Updates the {@link VisionSystemSim} with the latest robot pose.
     * This should be called periodically in sim only.
     */
    public void updateSim() {
        sim.update(simPoseSupplier.get());
    }

    /**
     * Updates the measurement consumer with the latest data from all cameras.
     * This should be called periodically.
     */
    public void updateVisionEstimates() {
        Pose2d prevEstimatedRobotPose = poseEstimator.getEstimatedPosition();

        final ChassisSpeeds speeds = chassisSpeedsSupplier.get();
        final double normSpeed = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond).getNorm();
        final boolean isAuto = DriverStation.isAutonomous();

        final Rotation2d heading = headingSupplier.get();
        final double now = Timer.getFPGATimestamp();

        for (int i = 0; i < cameras.length; i++) {
            AprilTagPhotonCamera photonCamera = cameras[i];

            photonCamera.addHeadingData(now, heading);
            photonCamera.update(prevEstimatedRobotPose, singleTagStdDevs,
                    isAuto ? multiTagStdDevs : multiTagTeleopStdDevs);

            List<Pair<EstimatedRobotPose, Matrix<N3, N1>>> robotPoses = photonCamera.getEstimatedRobotPoses();
            List<EstimatedRobotPose> trigPoses = photonCamera.getEstimatedTrigPoses();

            latestUsedPoses[i] = Pose3d.kZero;
            latestUsedTrigPoses[i] = Pose3d.kZero;

            for (Pair<EstimatedRobotPose, Matrix<N3, N1>> item : robotPoses) {
                EstimatedRobotPose estPose = item.getFirst();
                Matrix<N3, N1> stddevs = item.getSecond();

                latestUsedPoses[i] = estPose.estimatedPose;

                if (!isAuto || normSpeed < maxPoseEstimationChassisSpeed.in(MetersPerSecond)) {
                    visionMeasurementConsumer.accept(estPose.estimatedPose.toPose2d(), estPose.timestampSeconds,
                            stddevs);
                }
            }

            for (EstimatedRobotPose trigPose : trigPoses) {
                latestUsedTrigPoses[i] = trigPose.estimatedPose;

                if (!isAuto || normSpeed < maxPrecisePoseEstimationChassisSpeed.in(MetersPerSecond)) {
                    preciseVisionMeasurementConsumer.accept(
                            trigPose.estimatedPose.toPose2d(),
                            trigPose.timestampSeconds,
                            singleTagPreciseStdDevs);
                }
            }
        }
    }

    /**
     * Gets the supplied camera poses in the global frame, based off of the robot
     * pose.
     * 
     * @return the poses of each registered camera
     */
    @Log
    public Pose3d[] getCameraPoses() {
        if (poseEstimator == null) {
            return new Pose3d[0];
        }

        List<Pose3d> poses = new ArrayList<>();
        Pose2d robotPose = poseEstimator.getEstimatedPosition();
        for (AprilTagPhotonCamera camera : cameras) {
            poses.add(new Pose3d(robotPose).plus(camera.getRobotToCam()));
        }
        return poses.toArray(Pose3d[]::new);
    }

    /**
     * Gets the poses of all AprilTags in the current field layout.
     * 
     * @return the poses of all AprilTags
     */
    @Log
    public Pose3d[] getAprilTagPoses() {
        List<Pose3d> poses = new ArrayList<>();
        for (AprilTag tag : tagLayout.getTags()) {
            poses.add(tag.pose);
        }
        return poses.toArray(Pose3d[]::new);
    }

    /**
     * Gets an aggregated list of the latest cached measurements from all cameras,
     * so that they can be displayed easily. Does not actually update the cameras.
     * 
     * @return the latest cached measurements from all cameras
     */
    @Log
    public Pose3d[] getEstimatedRobotPoses() {
        List<Pose3d> poses = new ArrayList<>();
        for (AprilTagPhotonCamera camera : cameras) {
            poses.add(camera.getLoggedEstimatedRobotPose());
        }

        return poses.toArray(Pose3d[]::new);
    }

    /**
     * Gets an aggregated list of the latest cached trig measurements from all
     * cameras so that they can be displayed easily. Does not actually update the
     * cameras.
     * 
     * @return the latest cached measurements from all cameras
     */
    @Log
    public Pose3d[] getEstimatedPreciseRobotPoses() {
        List<Pose3d> poses = new ArrayList<>();
        for (AprilTagPhotonCamera camera : cameras) {
            poses.add(camera.getLoggedEstimatedTrigRobotPose());
        }

        return poses.toArray(Pose3d[]::new);
    }

    /**
     * Gets an aggregated list of the latest cached measurements from all cameras
     * so that they can be displayed easily. Does not actually update the cameras.
     * 
     * @return the latest cached measurements from all cameras
     */
    @Log
    public Pose3d[] getLastUsedRobotPoses() {
        return latestUsedPoses.clone();
    }

    /**
     * Gets an aggregated list of the latest cached trig measurements from all
     * cameras so that they can be displayed easily. Does not actually update the
     * cameras.
     * 
     * @return the latest cached measurements from all cameras
     */
    @Log
    public Pose3d[] getLastUsedPreciseRobotPoses() {
        return latestUsedTrigPoses.clone();
    }

    /**
     * Gets an aggregated list of all of the detected AprilTags from the latest
     * cached measurements from all cameras, so that they can be displayed easily.
     * Does not actually update the cameras.
     * 
     * @return the latest cached detected AprilTags from all cameras
     */
    @Log
    public Pose3d[] getDetectedAprilTags() {
        List<Pose3d> tags = new ArrayList<>();

        for (AprilTagPhotonCamera camera : cameras) {
            Collections.addAll(tags, camera.getLoggedDetectedAprilTags());
        }

        return tags.toArray(Pose3d[]::new);
    }
}
