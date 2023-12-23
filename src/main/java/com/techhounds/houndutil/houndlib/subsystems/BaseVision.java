package com.techhounds.houndutil.houndlib.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public interface BaseVision {
    public void updatePoseEstimator();

    public Pose3d[] getCameraPoses();

    public Pose3d[] getAprilTagPoses();

    public void setPoseEstimator(SwerveDrivePoseEstimator poseEstimator);

    public void setSimPoseSupplier(Supplier<Pose2d> simPoseSupplier);
}
