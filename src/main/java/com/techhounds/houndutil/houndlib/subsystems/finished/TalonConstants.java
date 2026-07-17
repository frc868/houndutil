package com.techhounds.houndutil.houndlib.subsystems.finished;

import java.util.ArrayList;
import java.util.Collections;

import com.ctre.phoenix6.signals.InvertedValue;

public class TalonConstants {
    private static final ArrayList<Integer> allCanIds = new ArrayList<Integer>();

    public final String SYSTEM_NAME;
    public final int CAN_ID;
    public final InvertedValue INVERT;

    public TalonConstants(String name, int canID, InvertedValue invert) {
        this.SYSTEM_NAME = name;
        this.CAN_ID = canID;
        this.INVERT = invert;

        allCanIds.add(this.CAN_ID);

        // TODO check if there are instances where multiple can be on a can ID
        if (Collections.frequency(allCanIds, this.CAN_ID) > 1) {
            // TODO check how this can be printed to driverstation or something
            System.out.println("\033[1m" + "WARNING: Can ID " + this.CAN_ID + " repeated. (Issued from TalonConstants.java)" + "\033[0m");
        }

    }
}
