package com.techhounds.houndutil.houndlib.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Base scaffolding for an shooter mechanism. Should use PID + Feedforward
 * control to spin at specific velocities. Use SI units
 * (for shooters, the most common SI unit would be radians/sec).
 */
public interface BaseShooter {
    /**
     * Gets the velocity of the flywheel. 0 should indicate it being stopped, and
     * the velocity should increase in the forward direction (i.e. the velocity
     * should be positive in the "correct" shooting direction).
     * 
     * @return the velocity of the flywheel, in rad/sec
     */
    public double getVelocity();

    /**
     * Explicit function to set the voltage of the motors attached to the elevator,
     * should handle safeties and clamping here.
     * 
     * @param voltage the voltage to apply to the motors, [-12, 12]
     */
    public void setVoltage(double voltage);

    /**
     * Creates a command that continuously spins the flywheel at a specific velocity
     * until cancelled. Note that this is *not* intended to self-cancel after
     * reaching its setpoint and defer to a default command.
     * 
     * @param goalVelocitySupplier a supplier of a velocity to spin at, in
     *                             radians/sec
     * @return the command
     */
    public Command spinAtVelocityCommand(Supplier<Double> goalVelocitySupplier);

    /**
     * Creates a command that manually sets the speed of the mechanism. Useful for
     * overriding PID control.
     * 
     * @apiNote use {@code setVoltage()} internally in order to maintain safeties
     *          and clamping
     * 
     * @param speed the speed [-1,1]
     * @return
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
