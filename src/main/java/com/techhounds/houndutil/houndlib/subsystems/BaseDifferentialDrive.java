package com.techhounds.houndutil.houndlib.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.path.PathPlannerPath;
import com.techhounds.houndutil.houndlib.MotorHoldMode;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Base scaffolding for a differential drivetrain. Intended to support pose
 * estimation, open and closed-loop driving, trajectory following, and
 * lower-level manual control modes. Use SI units throughout (for distance, use
 * meters, for velocity, use meters/sec, for rotation, use radians).
 */
public interface BaseDifferentialDrive {
    /**
     * The method with which to control the robot. ARCADE uses one joystick for
     * thrust (power), and one joystick for direction. TANK uses one joystick for
     * the left side, and one joystick for the right side.
     */
    public enum DifferentialDriveMode {
        ARCADE,
        TANK,
        CURVATURE;
    }

    /**
     * Gets the current (estimated) pose of the chassis, with respect to the origin.
     * 
     * @return the pose of the chassis
     */
    public Pose2d getPose();

    /**
     * Gets the current rotation of the chassis, with respect to the origin.
     * 
     * @return the rotation of the chassis, as a Rotation2d
     */
    public Rotation2d getRotation();

    /**
     * Gets the wheel positions of the chassis.
     * 
     * @return the wheel positions of the chassis
     */
    public DifferentialDriveWheelPositions getWheelPositions();

    /**
     * Gets the wheel speeds of the chassis.
     * 
     * @return the wheel speeds of the chassis
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds();

    /**
     * Gets the voltage output of the motors.
     * 
     * @return the voltage output of the motors
     */
    public DifferentialDriveWheelVoltages getWheelVoltages();

    /**
     * Gets the current <i>robot-relative</i> velocity of the chassis as a whole,
     * dependent on the states of the motors.
     * 
     * @return the velocity of the chassis, as a ChassisSpeeds
     */
    public ChassisSpeeds getChassisSpeeds();

    /**
     * Gets the pose estimator object for fusing latency-compensated vision
     * measurements with odometry data.
     * 
     * @return the SwerveDrivePoseEstimator object.
     */
    public DifferentialDrivePoseEstimator getPoseEstimator();

    /**
     * Updates the pose estimator with odometry data.
     */
    public void updatePoseEstimator();

    /**
     * Resets the pose estimator to a specific position on the field. Useful for
     * known starting locations before the autonomous period.
     * 
     * @param pose the pose to reset the chassis position to
     */
    public void resetPoseEstimator(Pose2d pose);

    /**
     * Resets the gyro such that the chassis is facing forward with respect to the
     * origin.
     */
    public void resetGyro();

    /**
     * Sets the motors into either brake or coast mode.
     * 
     * @param motorHoldMode the MotorHoldMode to set the motors to
     */
    public void setMotorHoldModes(MotorHoldMode motorHoldMode);

    /**
     * Sets the stator current limit on each motor. Useful for a temporary
     * high-power mode.
     * 
     * @param currentLimit the stator current limit to set, in amps
     */
    public void setCurrentLimit(int currentLimit);

    /**
     * Stops all motors.
     */
    public void stop();

    /**
     * Sets the states of the motors to accomplish the given chassis speeds.
     * 
     * @apiNote this should handle desaturation.
     * 
     * @param speeds the ChassisSpeeds to use to drive the chassis
     */
    public void drive(ChassisSpeeds speeds);

    /**
     * Sets the states of the motors to accomplish the given chassis speeds, with
     * closed-loop velocity control.
     * 
     * @apiNote this should handle desaturation.
     * 
     * @param speeds the ChassisSpeeds to use to drive the chassis
     */
    public void driveClosedLoop(ChassisSpeeds speeds);

    /**
     * Creates a command that moves the chassis given a left and right speed.
     * 
     * @apiNote this should handle deadbands, rate limiting, and any desired
     *          joystick curves.
     * @param leftStickThrustSupplier    the supplier for the left joystick thrust
     * @param rightStickThrustSupplier   the supplier for the right joystick thrust
     * @param rightStickRotationSupplier the supplier for the right joystick
     *                                   rotation (left-right)
     * @return the command
     */
    public Command teleopDriveCommand(DoubleSupplier leftStickThrustSupplier, DoubleSupplier rightStickThrustSupplier,
            DoubleSupplier rightStickRotationSupplier,
            Supplier<DifferentialDriveMode> driveModeSupplier);

    /**
     * Creates a command that follows a PathPlanner (or Choreo) path, then stops.
     * 
     * @param path the path to follow
     * @return the command
     */
    public Command followPathCommand(PathPlannerPath path);

    /**
     * Creates an instantaneous command that resets the gyro such that the chassis
     * is facing forward with respect to the origin.
     * 
     * @return the command
     */
    public Command resetGyroCommand();

    /**
     * Creates an instantaneous command that sets the stator current limit on the
     * motors.
     * 
     * @param currentLimit the stator current limit to set, in amps
     * @return the command
     */
    public Command setCurrentLimitCommand(int currentLimit);

    /**
     * Creates a command stops all motors and sets them to coast mode, to allow for
     * moving the chassis manually.
     * 
     * @apiNote use
     *          {@code .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)}
     *          for safety
     * @return the command
     */
    public Command coastMotorsCommand();
}
