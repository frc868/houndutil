package com.techhounds.houndutil.houndlib.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Current;

public class FinishedTalonSystem {
    public final int CAN_ID;
    public final InvertedValue INVERT;
    public final CANBus CANBUS;

    public FinishedTalonSystem(int canID, InvertedValue invert, Current currentLimit, double gearRatio, String canBus, boolean Brake){
        CAN_ID = canID;
        INVERT = invert;
        CANBUS = new CANBus(canBus);
    }
}

