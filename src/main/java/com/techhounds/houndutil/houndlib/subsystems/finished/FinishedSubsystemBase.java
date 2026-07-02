package com.techhounds.houndutil.houndlib.subsystems.finished;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class FinishedSubsystemBase extends SubsystemBase {
    public abstract FinishedTalonSystem[] getTalonInfo();
    public abstract boolean getAreFollowers();
    public abstract String getName();
    public abstract Current getCurrentLimit();
    public abstract double getGearRatio();
    public abstract NeutralModeValue getNeutral();
    public abstract KrakenType getKrakenType();
    public abstract MomentOfInertia getMomentOfInertia();
    public abstract CANBus getCanBus();
    public abstract double[] getTuningConstants();
}
