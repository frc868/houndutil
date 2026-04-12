/* MIT License

Copyright (c) 2025-2026 Littleton Robotics

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. */

package com.techhounds.houndutil.houndlib.commands;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.function.BooleanSupplier;

/**
 * A command composition that runs one of two commands, depending on the value
 * of the given
 * condition.
 *
 * <p>
 * Unlike the standard ConditionalCommand, this version evaluates the condition
 * continuously. If
 * the condition changes during execution, the currently running command is
 * interrupted (ended) and
 * the other command is initialized.
 *
 * <p>
 * The rules for command compositions apply: command instances that are passed
 * to it cannot be
 * added to any other composition or scheduled individually, and the composition
 * requires all
 * subsystems its components require.
 */
public class ContinuousConditionalCommand extends Command {
    private final Command onTrue;
    private final Command onFalse;
    private final BooleanSupplier condition;
    private Command selectedCommand;

    /**
     * Creates a new ContinuousConditionalCommand.
     *
     * @param onTrue    The command to run while the condition is true
     * @param onFalse   The command to run while the condition is false
     * @param condition The condition to determine which command to run
     */
    public ContinuousConditionalCommand(Command onTrue, Command onFalse, BooleanSupplier condition) {
        this.onTrue = requireNonNullParam(onTrue, "onTrue", "ConditionalCommand");
        this.onFalse = requireNonNullParam(onFalse, "onFalse", "ConditionalCommand");
        this.condition = requireNonNullParam(condition, "condition", "ConditionalCommand");

        CommandScheduler.getInstance().registerComposedCommands(onTrue, onFalse);

        addRequirements(onTrue.getRequirements());
        addRequirements(onFalse.getRequirements());
    }

    @Override
    public void initialize() {
        if (condition.getAsBoolean()) {
            selectedCommand = onTrue;
        } else {
            selectedCommand = onFalse;
        }
        selectedCommand.initialize();
    }

    @Override
    public void execute() {
        // Check condition to see if we need to swap commands
        boolean conditionState = condition.getAsBoolean();
        Command shouldBeRunning = conditionState ? onTrue : onFalse;

        if (selectedCommand != shouldBeRunning) {
            // Interrupt the current command
            selectedCommand.end(true);

            // Swap to the new command
            selectedCommand = shouldBeRunning;
            selectedCommand.initialize();
        }

        selectedCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        selectedCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return selectedCommand.isFinished();
    }

    @Override
    public boolean runsWhenDisabled() {
        return onTrue.runsWhenDisabled() && onFalse.runsWhenDisabled();
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        if (onTrue.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf
                || onFalse.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
            return InterruptionBehavior.kCancelSelf;
        } else {
            return InterruptionBehavior.kCancelIncoming;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addStringProperty("onTrue", onTrue::getName, null);
        builder.addStringProperty("onFalse", onFalse::getName, null);
        builder.addStringProperty(
                "selected",
                () -> {
                    if (selectedCommand == null) {
                        return "null";
                    } else {
                        return selectedCommand.getName();
                    }
                },
                null);
    }
}
