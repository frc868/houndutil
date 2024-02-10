package com.techhounds.houndutil.houndlib.subsystems;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.techhounds.houndutil.houndlib.MotorHoldMode;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;

public interface BaseSwerveDrive {
    public enum DriveMode {
        ROBOT_RELATIVE,
        FIELD_ORIENTED
    }

    public DriveMode getDriveMode();

    public Pose2d getPose();

    public Rotation2d getRotation();

    public SwerveModulePosition[] getModulePositions();

    public SwerveModuleState[] getModuleStates();

    public ChassisSpeeds getChassisSpeeds();

    public SwerveDrivePoseEstimator getPoseEstimator();

    public void updatePoseEstimator();

    public void resetPoseEstimator(Pose2d pose);

    public void resetGyro();

    public void setMotorHoldModes(MotorHoldMode motorHoldMode);

    public void setDriveCurrentLimit(int currentLimit);

    public void stop();

    public void setStates(SwerveModuleState[] state);

    public void setStatesClosedLoop(SwerveModuleState[] state);

    public void drive(ChassisSpeeds speeds);

    public void drive(ChassisSpeeds speeds, DriveMode driveMode);

    public void driveClosedLoop(ChassisSpeeds speeds, DriveMode driveMode);

    public Command teleopDriveCommand(DoubleSupplier xSpeedSupplier, DoubleSupplier ySpeedSupplier,
            DoubleSupplier thetaSpeedSupplier);

    public Command controlledRotateCommand(DoubleSupplier angle, DriveMode driveMode);

    public Command disableControlledRotateCommand();

    public Command wheelLockCommand();

    public Command turnWheelsToAngleCommand(double angle);

    public Command driveToPoseCommand(Pose2d pose);

    public Command followPathCommand(PathPlannerPath path);

    public Command driveDeltaCommand(Transform2d delta, PathConstraints constraints);

    public Command setDriveModeCommand(DriveMode driveMode);

    public Command resetGyroCommand();

    public Command setDriveCurrentLimitCommand(int currentLimit);

    public Command coastMotorsCommand();
}
