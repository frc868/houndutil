package com.techhounds.houndutil.houndlib.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * Base scaffolding for a swerve drivetrain. Intended to support pose
 * estimation, open and closed-loop driving, trajectory following, and
 * lower-level manual control modes.
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
     * Creates a command that moves the chassis given x, y, and theta speeds.
     * 
     * @apiNote this should handle deadbands, rate limiting, and any desired
     *          joystick curves. should also handle controlled rotation.
     * @param xSpeedSupplier     the supplier of the x speed
     * @param ySpeedSupplier     the supplier of the y speed
     * @param thetaSpeedSupplier the supplier of the θ speed
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
    public Command controlledRotateCommand(Supplier<Angle> angleSupplier);

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
     * @param angle the angle to turn the azimuth of the modules to
     * @return the command
     */
    public Command turnWheelsToAngleCommand(Angle angle);

    /**
     * Creates a command that drives the robot chassis to a specific pose. Does not
     * self-cancel when reaching the desired pose.
     * 
     * @apiNote this uses a Supplier so that the pose can change during the course
     *          of the command; useful for a lower-level version of a trajectory.
     * @param pose the supplier of the pose to drive the robot to
     * @return the command
     */
    public Command driveToPoseCommand(Supplier<Pose2d> poseSupplier);

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
     * @param currentLimit the stator current limit to set
     * @return the command
     */
    public Command setDriveCurrentLimitCommand(Current currentLimit);

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

    /**
     * Creates a command to run the drive SysId routine quasistatic.
     * @param direction the motor direction for the test
     * @return the command
     */
    public Command sysIdDriveQuasistaticCommand(SysIdRoutine.Direction direction);

    /**
     * Creates a command to run the drive SysId routine dynamic.
     * @param direction the motor direction for the test
     * @return the command
     */
    public Command sysIdDriveDynamicCommand(SysIdRoutine.Direction direction);

    /**
     * Creates a command to run the steer SysId routine quasistatic.
     * @param direction the motor direction for the test
     * @return the command
     */
    public Command sysIdSteerQuasistaticCommand(SysIdRoutine.Direction direction);

    /**
     * Creates a command to run the steer SysId routine dynamic.
     * @param direction the motor direction for the test
     * @return the command
     */
    public Command sysIdSteerDynamicCommand(SysIdRoutine.Direction direction);

}