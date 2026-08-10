package com.techhounds.houndutil.houndlib.subsystems.finished;

import java.util.List;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.VisionSystemSim;

import com.techhounds.houndutil.houndlib.TriConsumer;
import com.techhounds.houndutil.houndlib.vision.AprilTagPhotonCamera;
import com.techhounds.houndutil.houndlib.vision.AprilTagPhotonCamera.PhotonCameraConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class FinishedVision extends SubsystemBase {
    // Before reading this note that STD means standard deviation

    public static class StdDevs {
        public final Matrix<N3, N1> SINGLE;
        public final Matrix<N3, N1> MULTI;
        public final Matrix<N3, N1> TELEOP_MULTI;
        public final Matrix<N3, N1> SINGLE_PRECISE;

        public StdDevs(double single, double multi, double teleop, double precise) {
            this.SINGLE = VecBuilder.fill(single, single, Double.MAX_VALUE);
            this.MULTI = VecBuilder.fill(multi, multi, Double.MAX_VALUE);
            this.TELEOP_MULTI = VecBuilder.fill(teleop, teleop, Double.MAX_VALUE);
            this.SINGLE_PRECISE = VecBuilder.fill(precise, precise, Double.MAX_VALUE);
        }
    }

    private final Transform3d[] ROBOT_TO_CAMS;
    private final PhotonCameraConstants CAMERA_CONSTANTS;
    private final StdDevs STD_DEVS;
    private final AprilTagFieldLayout field;
    private final VisionSystemSim sim = new VisionSystemSim("main");
    private final Pose3d[] latestUsedPoses;
    private final Pose3d[] latestUsedTrigPoses;
    private final AprilTagPhotonCamera[] cameras;

    private final TriConsumer<Pose2d, Double, Matrix<N3, N1>> visionMeasurementConsumer;
    private final TriConsumer<Pose2d, Double, Matrix<N3, N1>> preciseVisionMeasurementConsumer;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Supplier<Pose2d> simPoseSupplier;
    private final Supplier<Rotation2d> headingSupplier;
    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

    public FinishedVision(PhotonCameraConstants cameraConstants, StdDevs stdDevs, AprilTagFieldLayout field,
            Transform3d[] robotToCams, FinishedDrivetrain drivetrain, double avgError, double stdDevError) {
        this.ROBOT_TO_CAMS = robotToCams;
        this.CAMERA_CONSTANTS = cameraConstants;
        this.STD_DEVS = stdDevs;
        this.field = field;

        this.cameras = new AprilTagPhotonCamera[ROBOT_TO_CAMS.length];
        this.latestUsedPoses = new Pose3d[cameras.length];
        this.latestUsedTrigPoses = new Pose3d[cameras.length];

        this.visionMeasurementConsumer = drivetrain.swerve::addVisionMeasurement;
        this.preciseVisionMeasurementConsumer = drivetrain.swerve::addPreciseVisionMeasurement;
        this.poseEstimator = drivetrain.swerve.getPoseEstimator();
        this.simPoseSupplier = drivetrain.swerve::getSimPose;
        this.headingSupplier = drivetrain.swerve::getRotation;
        this.chassisSpeedsSupplier = drivetrain.swerve::getChassisSpeeds;

        for (int i = 0; i < cameras.length; i++) {
            // TODO probably put avgError and stdDev into constants
            cameras[i] = new AprilTagPhotonCamera(String.valueOf(i), ROBOT_TO_CAMS[i], CAMERA_CONSTANTS, avgError,
                    stdDevError);
            latestUsedPoses[i] = Pose3d.kZero;
            latestUsedTrigPoses[i] = Pose3d.kZero;
        }
        if (RobotBase.isSimulation()) {
            sim.addAprilTags(field);
            for (AprilTagPhotonCamera camera : cameras) {
                sim.addCamera(camera.getSim(), camera.getRobotToCam());
            }
        }

    }

    /** Periodically update vision estimates */
    @Override
    public void periodic() {
        updateVisionEstimates();
    }

    /** Periodically update the sim */
    @Override
    public void simulationPeriodic() {
        sim.update(simPoseSupplier.get());
    }

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
            photonCamera.update(prevEstimatedRobotPose, STD_DEVS.SINGLE,
                    isAuto ? STD_DEVS.MULTI : STD_DEVS.TELEOP_MULTI);

            List<Pair<EstimatedRobotPose, Matrix<N3, N1>>> robotPoses = photonCamera.getEstimatedRobotPoses();
            List<EstimatedRobotPose> trigPoses = photonCamera.getEstimatedTrigPoses();

            latestUsedPoses[i] = Pose3d.kZero;
            latestUsedTrigPoses[i] = Pose3d.kZero;

            for (Pair<EstimatedRobotPose, Matrix<N3, N1>> item : robotPoses) {
                EstimatedRobotPose estPose = item.getFirst();
                Matrix<N3, N1> stddevs = item.getSecond();

                latestUsedPoses[i] = estPose.estimatedPose;

                if (!isAuto || normSpeed < 0.8) {
                    visionMeasurementConsumer.accept(estPose.estimatedPose.toPose2d(), estPose.timestampSeconds,
                            stddevs);
                }
            }

            for (EstimatedRobotPose trigPose : trigPoses) {
                latestUsedTrigPoses[i] = trigPose.estimatedPose;

                if (!isAuto || normSpeed < 2.0) {
                    preciseVisionMeasurementConsumer.accept(
                            trigPose.estimatedPose.toPose2d(),
                            trigPose.timestampSeconds,
                            STD_DEVS.SINGLE_PRECISE);
                }
            }
        }
    }

    public Pose3d[] getCameraPoses() {
        Pose3d[] x = new Pose3d[ROBOT_TO_CAMS.length];

        for (int i = 0; i < x.length; i++) {
            x[i] = new Pose3d(poseEstimator.getEstimatedPosition()).plus(ROBOT_TO_CAMS[i]);
        }

        return x;
    }

    public Pose3d[] getAprilTagPoses() {
        Pose3d[] x = new Pose3d[field.getTags().size()];

        for (int i = 0; i < x.length; i++) {
            x[i] = field.getTags().get(i).pose;
        }

        return x;
    }

    public Pose3d[] getEstimatedRobotPoses() {
        Pose3d[] x = new Pose3d[cameras.length];

        for (int i = 0; i < x.length; i++) {
            x[i] = cameras[i].getLoggedEstimatedRobotPose();
        }

        return x;
    }

    public Pose3d[] getEstimatedPreciseRobotPoses() {
        Pose3d[] x = new Pose3d[cameras.length];

        for (int i = 0; i < x.length; i++) {
            x[i] = cameras[i].getLoggedEstimatedTrigRobotPose();
        }

        return x;
    }

    public Pose3d[] getLastUsedRobotPoses() {
        return latestUsedPoses.clone();
    }

    public Pose3d[] getLastUsedPreciseRobotPoses() {
        return latestUsedTrigPoses.clone();
    }

    public Pose3d[] getDetectedAprilTags() {

        int totalSize = 0;

        for (AprilTagPhotonCamera cam : cameras) {
            totalSize += cam.getLoggedDetectedAprilTags().length;
        }

        Pose3d[] result = new Pose3d[totalSize];
        int index = 0;
        for (AprilTagPhotonCamera cam : cameras) {
            for (Pose3d tag : cam.getLoggedDetectedAprilTags()) {
                result[index++] = tag;
            }
        }

        return result;
    }

    // only for checking if poses are right in AdvantageScope
    public Pose3d[] getCameraPosesAgain() {
        return new Pose3d[] { // TODO make it work for N cameras
                new Pose3d().transformBy(ROBOT_TO_CAMS[0]),
                new Pose3d().transformBy(ROBOT_TO_CAMS[1]),
                new Pose3d().transformBy(ROBOT_TO_CAMS[2]),
                new Pose3d().transformBy(ROBOT_TO_CAMS[3])
        };
    }
}
