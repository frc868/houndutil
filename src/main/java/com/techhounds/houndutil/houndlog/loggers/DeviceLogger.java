package com.techhounds.houndutil.houndlog.loggers;

import com.techhounds.houndutil.houndlog.logitems.AbstractLogItem;

import java.util.Arrays;

import com.techhounds.houndutil.houndlog.LogProfileBuilder;

/**
 * A logger for a specific object. This logger will post all items contained
 * in {@code items} to SmartDashboard.
 * 
 * @author dr
 */
public class DeviceLogger extends Logger {
    /**
     * The name of the device to log.
     */
    private String deviceName;

    /**
     * An array of items to log. Using the unspecified generic form since this list
     * contains several types of {@link AbstractLogItem}.
     */
    protected AbstractLogItem<?>[] items;

    /**
     * Instantiate a DeviceLogger object.
     * 
     * @param obj        the object to log values for
     * @param subsystem  the name of the subsystem
     * @param deviceName the name of the device
     * @param items      the list of items to log (can xbe created manually or
     *                   through {@link LogProfileBuilder})
     */
    public DeviceLogger(String subsystem, String deviceName, AbstractLogItem<?>[] items) {
        this.deviceName = deviceName;
        this.items = items;
        setSubsystem(subsystem);
        setSubkeys();
    }

    /**
     * Instantiate a DeviceLogger object (without a subsystem, only use if setting
     * the subsystem through LogGroup).
     * 
     * @param obj        the object to log values for
     * @param deviceName the name of the device
     * @param items      the list of items to log (can be created manually or
     *                   through {@link LogProfileBuilder})
     */
    public DeviceLogger(String deviceName, AbstractLogItem<?>[] items) {
        this.deviceName = deviceName;
        this.items = items;
        setSubkeys();
    }

    @Override
    public void setSubsystem(String subsystem) {
        for (Logger logger : items) {
            logger.setSubsystem(subsystem);
        }
    }

    public void setSubkeys() {
        for (AbstractLogItem<?> item : items) {
            item.setSubkeys(Arrays.asList(deviceName));
        }
    }

    /**
     * Does nothing, because we don't need to init for a {@code DeviceLogger}.
     */
    @Override
    public void init() {

    }

    /**
     * Logs each item in {@code items}.
     */
    @Override
    public void run() {
        for (AbstractLogItem<?> item : items) {
            item.run();
        }
    }
}
