package com.techhounds.houndutil.houndlib;

import java.util.List;
import java.util.stream.Collectors;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class AdvantageScopeSerializer {
    public static double[] serializeSwerveModuleStates(SwerveModuleState[] states) {
        double[] output = new double[states.length * 2];
        for (int i = 0; i < states.length * 2; i += 2) {
            output[i] = states[i / 2].angle.getDegrees();
            output[i + 1] = states[i / 2].speedMetersPerSecond;
        }
        return output;
    }

    public static double[] serializeAprilTags(List<AprilTag> tags) {
        return serializePose3ds(tags.stream().map(tag -> tag.pose).collect(Collectors.toList()));
    }

    public static double[] serializePose3ds(List<Pose3d> poses) {
        double[] poseData = new double[poses.size() * 7];
        for (int i = 0; i < poses.size() * 7; i += 7) {
            Pose3d tagPose = poses.get(i / 7);
            poseData[i] = tagPose.getX();
            poseData[i + 1] = tagPose.getY();
            poseData[i + 2] = tagPose.getZ();
            Quaternion tagQuat = tagPose.getRotation().getQuaternion();
            poseData[i + 3] = tagQuat.getW();
            poseData[i + 4] = tagQuat.getX();
            poseData[i + 5] = tagQuat.getY();
            poseData[i + 6] = tagQuat.getZ();
        }
        return poseData;
    }

    public static double[] serializePose2ds(List<Pose2d> poses) {
        double[] poseData = new double[poses.size() * 3];
        for (int i = 0; i < poses.size() * 3; i += 3) {
            Pose2d pose = poses.get(i / 3);
            poseData[i] = pose.getX();
            poseData[i + 1] = pose.getY();
            poseData[i + 2] = pose.getRotation().getDegrees();
        }
        return poseData;
    }

    public static double[] serializeAprilTagIDs(List<AprilTag> tags) {
        double[] idData = new double[tags.size()];
        for (int i = 0; i < tags.size(); i++) {
            idData[i] = tags.get(i).ID;
        }
        return idData;
    }
}
