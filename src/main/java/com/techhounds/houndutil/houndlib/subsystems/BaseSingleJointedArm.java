package com.techhounds.houndutil.houndlib.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Base scaffolding for an single-jointed arm mechanism. Should use PID or
 * profiled PID
 * control to move between setpoints or to an arbitrary position. Use SI units
 * (for arms, the most common SI unit is radians).
 * <p>
 * To use, implement {@code BaseSingleJointedArm<MechanismPosition>}, where
 * {@code MechanismPosition} is an enum with the various setpoints that can be
 * used.
 * <p>
 * Examples of single-jointed arm-type mechanisms include: typical arms,
 * turrets, and rotational-deployment intakes
 */
public interface BaseSingleJointedArm<T extends Enum<T>> {
    /**
     * Gets the position of the mechanism.
     * 
     * <p>
     * 0 should be <i>at the point where the arm is parallel to the ground</i>, or
     * where the force of gravity is strongest. In CAD, this looks like the line
     * between
     * the pivot and the center of mass of the arm being parallel to the ground. The
     * position should increase as the arm moves up, and decrease as the arm moves
     * down.
     * <p>
     * Note that this calibration is not necessary if the arm is horizontal.
     * 
     * @return the position of the mechanism, in meters
     */
    double getPosition();

    /**
     * Resets the position of the mechanism to a specific value (this should be the
     * position of a hard stop).
     */
    void resetPosition();

    /**
     * Explicit function to set the voltage of the motors attached to the arm,
     * should handle safeties and clamping here.
     * 
     * @param voltage the voltage to apply to the motors, [-12, 12]
     */
    void setVoltage(double voltage);

    /**
     * Creates a command that continuously applies voltage to the motor controllers
     * to move them to the currently set goal.
     * 
     * @return the command
     */
    public Command moveToCurrentGoalCommand();

    /**
     * Creates a command that sets the current goal position to the setpoint, and
     * cancels once the mechanism has reached that goal.
     * 
     * @apiNote use {@code moveToCurrentGoalCommand()} internally to avoid code
     *          duplication
     * 
     * @param goalPositionSupplier a supplier of an instance of the setpoint enum
     * @return the command
     */
    public Command moveToPositionCommand(Supplier<T> goalPositionSupplier);

    /**
     * Creates a command that sets the current goal position to the setpoint, and
     * cancels once the mechanism has reached that goal.
     * 
     * @apiNote use {@code moveToCurrentGoalCommand()} internally to avoid code
     *          duplication
     * 
     * @param goalPositionSupplier a supplier of a position to move to, in meters
     * @return the command
     */
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier);

    /**
     * Creates a command that sets the current goal position to the setpoint plus
     * the delta (if a delta of pi/4 is set, the arm should move up 45 degrees), and
     * cancels once the mechanism has reached that goal.
     * 
     * @apiNote use {@code moveToCurrentGoalCommand()} internally to avoid code
     *          duplication
     * 
     * @param delta a supplier of a delta to move, in radians
     * @return the command
     */
    public Command movePositionDeltaCommand(Supplier<Double> delta);

    /**
     * Creates a command that sets the goal to the current position, and moves to
     * that goal until cancelled.
     * 
     * @return the command
     */
    public Command holdCurrentPositionCommand();

    /**
     * Creates an instantaneous command that resets the position of the arm.
     * 
     * @return the command
     */
    public Command resetPositionCommand();

    /**
     * Creates a command that manually sets the speed of the mechanism. Useful for
     * overriding PID control.
     * 
     * @apiNote use {@code setVoltage()} internally in order to maintain safeties
     *          and clamping
     * 
     * @param speed the speed [-1,1]
     * @return the command
     */
    public Command setOverridenSpeedCommand(Supplier<Double> speed);

    /**
     * Creates a command stops the motor and sets it to coast mode, to allow for
     * moving the mechanism manually.
     * 
     * @apiNote use
     *          {@code .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)}
     *          for safety
     * @return the command
     */
    public Command coastMotorsCommand();
}
