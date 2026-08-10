package com.techhounds.houndutil.houndlib.subsystems.finished;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.ControlRequest;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;

public interface FinishedJavadocs<T extends Measure<?>> {
    /**
     * stops the system
     */
    default void stop() {
        throw new UnsupportedOperationException("The 'stop' operation is not supported by this subsystem.");
    }

    /**
     * @return a {@code Command} to stop the system
     */
    default Command stopCommand() {
        throw new UnsupportedOperationException("The 'stopCommand' operation is not supported by this subsystem.");
    }

    /**
     * @param goalVelocitySupplier a supplier of a velocity to spin at
     * @return a {@code Command} to run the flywheel at a given angular velocity
     *         until interrupted
     */
    default Command spinAtVelocityCommand(Supplier<AngularVelocity> x) {
        throw new UnsupportedOperationException(
                "The 'spinAtVelocityCommand' operation is not supported by this subsystem.");
    }

    /**
     * Creates a command stops the motor and sets it to coast mode, to allow for
     * moving the mechanism manually. Cancels incoming commands for safety.
     * 
     * @return the command
     */
    default Command coastMotorsCommand() {
        throw new UnsupportedOperationException(
                "The 'coastMotorsCommand' operation is not supported by this subsystem.");
    }

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
    default Command setOverridenSpeedCommand(Supplier<Double> x) {
        throw new UnsupportedOperationException(
                "The 'setOverridenSpeedCommand' operation is not supported by this subsystem.");
    }

    /**
     * Gets the velocity of the system, where negative values are allowed,
     * representing reversed movement.
     * 
     * @return the velocity
     */
    default T getVelocity() {
        throw new UnsupportedOperationException(
                "The 'getVelocity' operation is not supported by this subsystem.");
    }

    /**
     * sets the controller of all the motors, mainly used internally in houndutil
     * 
     * @param control the request to control the motors
     */
    default void setMotorsControl(ControlRequest x) {
        throw new UnsupportedOperationException(
                "The 'setMotorsControl' operation is not supported by this subsystem.");
    }

    /**
     * Creates a command that runs the rollers of the intake at a specific voltage.
     * A negative voltage moves it in the opposite direction.
     * 
     * @return the command
     */
    default Command runRollersCommand(Voltage x) {
        throw new UnsupportedOperationException(
                "The 'runRollersCommand' operation is not supported by this subsystem.");
    }

    /**
     * tells mechanism it is zeroed and sets {@code initialized = true}
     */
    default void resetPosition() {
        throw new UnsupportedOperationException(
                "The 'resetPosition' operation is not supported by this subsystem.");
    }

    /**
     * @return a command to tell mechanism it is zeroed and to set
     *         {@code initialized = true}
     */
    default Command resetPositionCommand() {
        throw new UnsupportedOperationException(
                "The 'resetPositionCommand' operation is not supported by this subsystem.");
    }

    /**
     * Gets the position of the system
     * 
     * @return the position
     */
    default T getPosition() {
        throw new UnsupportedOperationException(
                "The 'getPosition' operation is not supported by this subsystem.");
    }

    /**
     * Creates a command that continuously applies voltage to the motor controllers
     * to move them to the currently set goal.
     * 
     * @return the command
     */
    default Command moveToCurrentGoalCommand() {
        throw new UnsupportedOperationException(
                "The 'moveToCurrentGoalCommand' operation is not supported by this subsystem.");
    }

    /**
     * @param goalPositionSupplier a supplier of a position to move to
     * @return a command that sets the current goal position to the setpoint.
     */
    default Command moveToArbitraryPositionCommand(Supplier<T> x) {
        throw new UnsupportedOperationException(
                "The 'moveToArbitraryPositionCommand' operation is not supported by this subsystem.");
    }

    /**
     * Creates a command that sets the current goal position to the current position
     * plus a delta then moves to that goal until it reaches it.
     * 
     * @param delta a supplier of a delta to move
     * @return the command
     */
    default Command movePositionDeltaCommand(Supplier<T> x) {
        throw new UnsupportedOperationException(
                "The 'movePositionDeltaCommand' operation is not supported by this subsystem.");
    }

    /**
     * @return a boolean describing whether the system has reached the goal or not
     */
    default boolean atGoal() {
        throw new UnsupportedOperationException(
                "The 'atGoal' operation is not supported by this subsystem.");
    }

}
