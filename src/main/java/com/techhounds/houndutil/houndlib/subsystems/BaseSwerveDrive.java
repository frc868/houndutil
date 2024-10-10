package com.techhounds.houndutil.houndlib.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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

/**
 * Base scaffolding for a swerve drivetrain. Intended to support pose
 * estimation, open and closed-loop driving, trajectory following, and
 * lower-level manual control modes. Use SI units throughout (for distance, use
 * meters, for velocity, use meters/sec, and for the azimuth of the wheels, use
 * radians).
 * 
 * <br>
 * </br>
 * 
 * <h2>Conventions</h2>
 * 
 * The origin is the furthest right point on the blue alliance
 * wall when looking at the field from behind the driver stations, with the
 * chassis rotation facing the red alliance wall. This point is marked (0, 0,
 * 0).
 * 
 * <pre>
 *┌───────────────────────────┐
 *│ B                     R   │
 *│ L                     E   │
 *│ U                     D   │
 *│ E                         │
 *X───────────────────────────┘
 * </pre>
 * 
 * <h2>Coordinates</h2>
 * 
 * The field coordinate system uses (x, y, θ), where the positive x direction is
 * towards the red alliance wall, and the positive y direction is towards the
 * opposite field wall to the left.
 * 
 * <pre>
 *┌───────────────────────────┐
 *│                           │
 *+y                          │
 *↑                           │
 *│                           │
 *•───→+x─────────────────────┘
 * </pre>
 * 
 */
public interface BaseSwerveDrive {
    /**
     * The orientation in which to move the drivetrain. Robot-relative indicates
     * that forward translations will move the drivetrain in the direction of the
     * front of the chassis. Field-oriented indicates that forward translations will
     * move the drivetrain away from the driver, regardless of the chassis
     * orientation. The gyro must be set correctly for the field-oriented mode to
     * work properly.
     */
    public enum DriveMode {
        ROBOT_RELATIVE,
        FIELD_ORIENTED
    }

    /**
     * Gets the currently set drive mode.
     * 
     * @return the drive mode
     */
    public DriveMode getDriveMode();

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
     * Gets an array containing the distance travelled and the azimuth angle for
     * each swerve module. Used mainly for odometry.
     * 
     * @return the array of SwerveModulePositions
     */
    public SwerveModulePosition[] getModulePositions();

    /**
     * Gets an array containing the current velocity and the azimuth angle for
     * each swerve module.
     * 
     * @return the array of SwerveModuleStates
     */
    public SwerveModuleState[] getModuleStates();

    /**
     * Gets the current <i>robot-relative</i> velocity of the chassis as a whole,
     * dependent on the states of the swerve modules.
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
    public SwerveDrivePoseEstimator getPoseEstimator();

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
     * Sets the motors on the swerve modules into either brake or coast mode.
     * 
     * @param motorHoldMode the MotorHoldMode to set the motors in each swerve
     *                      module to
     */
    public void setMotorHoldModes(MotorHoldMode motorHoldMode);

    /**
     * Sets the stator current limit on each swerve module. Useful for a temporary
     * high-power mode.
     * 
     * @param currentLimit the stator current limit to set, in amps
     */
    public void setDriveCurrentLimit(int currentLimit);

    /**
     * Stops all swerve modules.
     */
    public void stop();

    /**
     * Sets the state (velocity and azimuth angle) of each swerve module, without
     * closed-loop velocity control.
     * 
     * @apiNote use for standard tele-operated driving
     * @param state an array of SwerveModuleStates to set the modules to
     */
    public void setStates(SwerveModuleState[] state);

    /**
     * Sets the state (velocity and azimuth angle) of each swerve module, with
     * closed-loop velocity control.
     * 
     * @apiNote use for trajectory following
     * @param state an array of SwerveModuleStates to set the modules to
     */
    public void setStatesClosedLoop(SwerveModuleState[] state);

    /**
     * Sets the states of the swerve modules to accomplish the given chassis speeds.
     * Uses the currently set DriveMode.
     * 
     * @apiNote this should handle discretization, desaturation, and optimization.
     * 
     * @param speeds the ChassisSpeeds to use to drive the swerve modules
     */
    public void drive(ChassisSpeeds speeds);

    /**
     * Sets the states of the swerve modules to accomplish the given chassis speeds.
     * 
     * @apiNote this should handle discretization, desaturation, and optimization.
     * 
     * @param speeds    the ChassisSpeeds to use to drive the swerve modules.
     * @param driveMode the DriveMode to use for the chassis' reference point
     */
    public void drive(ChassisSpeeds speeds, DriveMode driveMode);

    /**
     * Sets the states of the swerve modules to accomplish the given chassis speeds,
     * with closed-loop velocity control.
     * 
     * @apiNote this should handle discretization, desaturation, and optimization.
     * 
     * @param speeds    the ChassisSpeeds to use to drive the swerve modules.
     * @param driveMode the DriveMode to use for the chassis' reference point
     */
    public void driveClosedLoop(ChassisSpeeds speeds, DriveMode driveMode);

    /**
     * Creates a command that moves the chassis given x, y, and theta speeds.
     * 
     * @apiNote this should handle deadbands, rate limiting, and any desired
     *          joystick curves. should also handle controlled rotation.
     * @param xSpeedSupplier     the supplier of the x speed, in m/s
     * @param ySpeedSupplier     the supplier of the y speed, in m/s
     * @param thetaSpeedSupplier the supplier of the θ speed, in rad/s
     * @return the command
     */
    public Command teleopDriveCommand(DoubleSupplier xSpeedSupplier, DoubleSupplier ySpeedSupplier,
            DoubleSupplier thetaSpeedSupplier);

    /**
     * Creates an instantaneous command that enables motion-profiled rotation of the
     * chassis to a specific angle.
     * 
     * @param angle the angle to rotate the chassis to
     * @return the command
     */
    public Command controlledRotateCommand(DoubleSupplier angle);

    /**
     * Creates an instantaneous command that disables motion-profiled rotation of
     * the chassis.
     * 
     * @return the command
     */
    public Command disableControlledRotateCommand();

    /**
     * Creates a command that sets the velocities of each swerve module to 0, and
     * rotates the wheels to an X pattern to resist pushing or other movement.
     * 
     * @return the command
     */
    public Command wheelLockCommand();

    /**
     * Creates a command that turns all of the swerve modules to a specific azimuth
     * angle.
     * 
     * @param angle the angle to turn the azimuth of the modules to, in radians
     * @return the command
     */
    public Command turnWheelsToAngleCommand(double angle);

    /**
     * Creates a command that drives the robot chassis to a specific pose. Does not
     * self-cancel when reaching the desired pose.
     * 
     * @apiNote this uses a Supplier so that the pose can change during the course
     *          of the command; useful for a lower-level version of a trajectory.
     * @param pose the supplier of the pose to drive the robot to
     * @return the command
     */
    public Command driveToPoseCommand(Supplier<Pose2d> pose);

    /**
     * Creates a command that follows a PathPlanner (or Choreo) path, then stops.
     * 
     * @param path the path to follow
     * @return the command
     */
    public Command followPathCommand(PathPlannerPath path);

    /**
     * Creates a command that moves the robot through a specific transform. Uses
     * specific constraints to generate an on-the-fly trajectory.
     * 
     * @param delta the amount to move the robot given the current pose. note that
     *              this applies as a transformation to the current pose, so a
     *              Transform2d with a positive X position will result in
     *              robot-relative movement.
     * @return the command
     */
    public Command driveDeltaCommand(Transform2d delta, PathConstraints constraints);

    /**
     * Creates an instantaneous command that sets the current driving mode.
     * 
     * @param driveMode the DriveMode to set
     * @return the command
     */
    public Command setDriveModeCommand(DriveMode driveMode);

    /**
     * Creates an instantaneous command that resets the gyro such that the chassis
     * is facing
     * forward with respect to the origin.
     * 
     * @return the command
     */
    public Command resetGyroCommand();

    /**
     * Creates an instantaneous command that sets the stator current limit on each
     * swerve module.
     * 
     * @param currentLimit the stator current limit to set, in amps
     * @return the command
     */
    public Command setDriveCurrentLimitCommand(int currentLimit);

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
