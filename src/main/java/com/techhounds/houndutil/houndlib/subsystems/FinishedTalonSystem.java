package com.techhounds.houndutil.houndlib.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Current;

public class FinishedTalonSystem {
    public final int CAN_ID;
    public final InvertedValue INVERT;
    public final Current CURRENT_LIMIT;
    public final double GEAR_RATIO;
    public final CANBus CANBUS;

    public FinishedTalonSystem(int canID, boolean clockwise, double currentLimit, double gearRatio, String canBus){
        CAN_ID = canID;
        INVERT = clockwise ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        CURRENT_LIMIT = Amps.of(currentLimit);
        GEAR_RATIO = gearRatio;
        CANBUS = new CANBus(canBus);
    }
}
