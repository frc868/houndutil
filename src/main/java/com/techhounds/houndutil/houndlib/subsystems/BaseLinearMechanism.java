package com.techhounds.houndutil.houndlib.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Base scaffolding for a linear mechanism. Should use PID or profiled PID
 * control to move between setpoints or to an arbitrary position. Use SI units
 * (for linear mechanisms, the most common SI unit is meters).
 * <p>
 * To use, implement {@code BaseLinearMechanism<MechanismPosition>}, where
 * {@code MechanismPosition} is an enum with the various setpoints that can be
 * used.
 * <p>
 * Examples of "linear mechanism"-type mechanisms include: elevators, climbers,
 * lead screws, telescoping tube, and linear intakes
 */
public interface BaseLinearMechanism<T extends Enum<T>> {
    /**
     * Gets the position of the mechanism. 0 should be at the lowest movement point,
     * and the position should increase as the mechanism moves up.
     * 
     * @return the position of the mechanism, in meters
     */
    public double getPosition();

    /**
     * Resets the position of the mechanism to a specific value (this should be the
     * position of a hard stop).
     */
    public void resetPosition();

    /**
     * Explicit function to set the voltage of the motors attached to the linear
     * mechanism,
     * should handle safeties and clamping here.
     * 
     * @param voltage the voltage to apply to the motors, [-12, 12]
     */
    public void setVoltage(double voltage);

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
     * the delta (if a delta of 0.1 is set, the linear mechanism should move up
     * 10cm), and
     * cancels once the mechanism has reached that goal.
     * 
     * @apiNote use {@code moveToCurrentGoalCommand()} internally to avoid code
     *          duplication
     * 
     * @param delta a supplier of a delta to move, in meters
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
     * Creates an instantaneous command that resets the position of the linear
     * mechanism.
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
