package com.techhounds.houndutil.houndlog;

import java.util.ArrayList;

import com.ctre.phoenix6.BaseStatusSignal;

public class SignalManager {
    private static ArrayList<BaseStatusSignal> statusesList = new ArrayList<>();
    private static BaseStatusSignal[] statuses;

    public static void register(BaseStatusSignal... statuses) {
        for (var status : statuses) {
            statusesList.add(status);
        }
    }

    public static void finalizeAll() {
        statuses = new BaseStatusSignal[statusesList.size()];

        statusesList.toArray(statuses);

        statusesList.clear();
    }

    public static void refresh() {
        BaseStatusSignal.waitForAll(0, statuses);
    }
}