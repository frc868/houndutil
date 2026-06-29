package com.techhounds.houndutil.houndlib.subsystems;

import com.ctre.phoenix6.signals.InvertedValue;

public class FinishedTalonSystem {
    public final String SYSTEM_NAME;
    public final int CAN_ID;
    public final InvertedValue INVERT;

    public FinishedTalonSystem(String name, int canID, InvertedValue invert){
        SYSTEM_NAME = name;
        CAN_ID = canID;
        INVERT = invert;
    }
}

