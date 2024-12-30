package com.techhounds.houndutil.houndlog;

import java.util.ArrayList;

import com.ctre.phoenix6.BaseStatusSignal;

/**
 * Manages CTRE status signals, used to reduce performance overhead of reading
 * status signals from multiple devices.
 */
public class SignalManager {
    private static ArrayList<BaseStatusSignal> statusesList = new ArrayList<>();
    private static BaseStatusSignal[] statuses;

    /**
     * Registers status signals to be updated by the SignalManager.
     * 
     * @param statuses the status signals to register
     */
    public static void register(BaseStatusSignal... statuses) {
        for (var status : statuses) {
            statusesList.add(status);
        }
    }

    /**
     * Finalizes all registered status signals. This must be called after all status
     * signals have been registered.
     */
    public static void finalizeAll() {
        statuses = new BaseStatusSignal[statusesList.size()];

        statusesList.toArray(statuses);

        statusesList.clear();
    }

    /**
     * Refreshes all registered status signals. This should be called periodically.
     */
    public static void refresh() {
        if (statuses.length > 0)
            BaseStatusSignal.waitForAll(0, statuses);
    }
}