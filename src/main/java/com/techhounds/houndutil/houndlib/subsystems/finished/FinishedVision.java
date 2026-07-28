package com.techhounds.houndutil.houndlib.subsystems.finished;

import java.util.function.Supplier;

import org.photonvision.simulation.VisionSystemSim;

import com.techhounds.houndutil.houndlib.TriConsumer;
import com.techhounds.houndutil.houndlib.vision.AprilTagPhotonCamera;
import com.techhounds.houndutil.houndlib.vision.AprilTagPhotonCamera.PhotonCameraConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class FinishedVision extends SubsystemBase{
    //Before reading this note that SD means standard deviation

    public class StdDevs{
        public final Matrix<N3, N1> SINGLE;
        public final Matrix<N3, N1> MULTI;
        public final Matrix<N3, N1> TELEOP_MULTI;
        public final Matrix<N3, N1> SINGLE_PRECISE;
        public StdDevs(double single, double multi, double teleop, double precise){
            this.SINGLE = VecBuilder.fill(single, single, Double.MAX_VALUE);
            this.MULTI = VecBuilder.fill(multi, multi, Double.MAX_VALUE);
            this.TELEOP_MULTI = VecBuilder.fill(teleop, teleop, Double.MAX_VALUE);
            this.SINGLE_PRECISE = VecBuilder.fill(precise, precise, Double.MAX_VALUE);
        }
    }

    private final Transform3d[] ROBOT_TO_CAM;
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


    public FinishedVision(PhotonCameraConstants cameraConstants, StdDevs stdDevs, AprilTagFieldLayout field, Transform3d[] robotToCam, FinishedDrivetrain drivetrain, double avgError, double stdDevError) {
        this.ROBOT_TO_CAM = robotToCam;
        this.CAMERA_CONSTANTS = cameraConstants;
        this.STD_DEVS = stdDevs;
        this.field = field;


        this.cameras = new AprilTagPhotonCamera[ROBOT_TO_CAM.length];
        this.latestUsedPoses = new Pose3d[cameras.length];
        this.latestUsedTrigPoses = new Pose3d[cameras.length];

        this.visionMeasurementConsumer = drivetrain.swerve::addVisionMeasurement;
        this.preciseVisionMeasurementConsumer = drivetrain.swerve::addPreciseVisionMeasurement;
        this.poseEstimator = drivetrain.swerve.getPoseEstimator();
        this.simPoseSupplier = drivetrain.swerve::getSimPose;
        this.headingSupplier = drivetrain.swerve::getRotation;
        this.chassisSpeedsSupplier = drivetrain.swerve::getChassisSpeeds;
        


        for (int i = 0; i < cameras.length; i++) {
            //TODO probably put avgError and stdDev into constants
            cameras[i] = new AprilTagPhotonCamera(String.valueOf(i), robotToCam[i], cameraConstants, avgError, stdDev); 
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
}
