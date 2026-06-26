package com.techhounds.houndutil.houndlib.subsystems;

import com.ctre.phoenix6.signals.InvertedValue;

public class FinishedTalonSystem {
    public final int CAN_ID;
    public final InvertedValue INVERT;

    public FinishedTalonSystem(int canID, InvertedValue invert){
        CAN_ID = canID;
        INVERT = invert;
    }
}

