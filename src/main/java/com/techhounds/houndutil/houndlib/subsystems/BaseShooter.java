package com.techhounds.houndutil.houndlib.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

public interface BaseShooter {
    double getVelocity();

    void setVoltage(double voltage);

    public Command spinAtVelocityCommand(Supplier<Double> goalVelocitySupplier);

    public Command setOverridenSpeedCommand(Supplier<Double> speed);

    public Command coastMotorsCommand();
}
