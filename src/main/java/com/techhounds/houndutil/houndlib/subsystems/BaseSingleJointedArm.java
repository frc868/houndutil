package com.techhounds.houndutil.houndlib.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

public interface BaseSingleJointedArm<T extends Enum<T>> {
    double getPosition();

    void resetPosition();

    void setVoltage(double voltage);

    public Command moveToCurrentGoalCommand();

    public Command moveToPositionCommand(Supplier<T> goalPositionSupplier);

    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier);

    public Command movePositionDeltaCommand(Supplier<Double> delta);

    public Command holdCurrentPositionCommand();

    public Command resetPositionCommand();

    public Command setOverridenSpeedCommand(Supplier<Double> speed);

    public Command coastMotorsCommand();
}
