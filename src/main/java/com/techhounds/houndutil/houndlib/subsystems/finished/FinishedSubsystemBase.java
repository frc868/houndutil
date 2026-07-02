package com.techhounds.houndutil.houndlib.subsystems.finished;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;

public interface FinishedSubsystemBase{
    public FinishedTalonSystem[] getTalonInfo();
    public boolean getAreFollowers();
    public String getName();
    public Current getCurrentLimit();
    public double getGearRatio();
    public NeutralModeValue getNeutral();
    public KrakenType getKrakenType();
    public MomentOfInertia getMomentOfInertia();
    public CANBus getCanBus();
    public double[] getTuningConstants();
}
