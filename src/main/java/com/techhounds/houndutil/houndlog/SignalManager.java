package com.techhounds.houndutil.houndlog;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;

import com.ctre.phoenix6.BaseStatusSignal;

/**
 * Manages CTRE status signals, used to reduce performance overhead of reading
 * status signals from multiple devices.
 */
public class SignalManager {
    // private static ArrayList<BaseStatusSignal> statusesList = new ArrayList<>();
    private static Map<String, ArrayList<BaseStatusSignal>> statusMap = new HashMap<>();
    private static BaseStatusSignal[][] finalizedStatusMap;

    /**
     * Registers status signals to be updated by the SignalManager.
     * 
     * @param statuses the status signals to register
     */
    public static void register(String bus, BaseStatusSignal... statuses) {
        statusMap.putIfAbsent(bus, new ArrayList<>());

        for (var status : statuses) {
            statusMap.get(bus).add(status);
        }
    }

    /**
     * Finalizes all registered status signals. This must be called after all status
     * signals have been registered.
     */
    public static void finalizeAll() {
        int keys = statusMap.keySet().size();
        finalizedStatusMap = new BaseStatusSignal[keys][];
        AtomicInteger index = new AtomicInteger(0);
        statusMap.keySet().forEach((k) -> {
            finalizedStatusMap[index.get()] = new BaseStatusSignal[statusMap.get(k).size()];
            statusMap.get(k).toArray(finalizedStatusMap[index.getAndIncrement()]);
            statusMap.get(k).clear();
        });
    }

    /**
     * Refreshes all registered status signals. This should be called periodically.
     */
    public static void refresh() {
        for (int i = 0; i < finalizedStatusMap.length; i++) {
            if (finalizedStatusMap[i].length > 0) {
                BaseStatusSignal.waitForAll(0, finalizedStatusMap[i]);
            }
        }
    }
}