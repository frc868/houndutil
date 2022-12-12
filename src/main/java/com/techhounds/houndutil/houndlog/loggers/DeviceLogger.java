package com.techhounds.houndutil.houndlog.loggers;

import edu.wpi.first.networktables.NetworkTable;
import com.techhounds.houndutil.houndlog.logitems.LogItem;
import com.techhounds.houndutil.houndlog.LogProfileBuilder;

/**
 * A logger for a specified object T. This logger will post all items contained
 * in {@code items} to SmartDashboard.
 * 
 * @author dr
 */
public class DeviceLogger<T> extends Logger {
    /**
     * The object we are logging.
     */
    public T obj;

    /**
     * The name of the device to log.
     */
    private String deviceName;

    /**
     * An array of items to log. Using the unspecified generic form since this list
     * contains several types of {@link LogItem}.
     */
    protected LogItem<?>[] items;

    /**
     * Instantiate a DeviceLogger object.
     * 
     * @param obj        the object to log values for
     * @param subsystem  the name of the subsystem
     * @param deviceName the name of the device
     * @param items      the list of items to log (can be created manually or
     *                   through {@link LogProfileBuilder})
     */
    public DeviceLogger(T obj, String subsystem, String deviceName, LogItem<?>[] items) {
        super(subsystem);
        this.obj = obj;
        this.deviceName = deviceName;
        this.items = items;

        for (LogItem<?> i : items) {
            i.setDataTable(getDataTable());
        }
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
    public DeviceLogger(T obj, String deviceName, LogItem<?>[] items) {
        this.obj = obj;
        this.deviceName = deviceName;
        this.items = items;

        for (LogItem<?> i : items) {
            i.setDataTable(getDataTable());
        }
    }

    /**
     * Gets the data table associated with the device.
     */
    @Override
    protected NetworkTable getDataTable() {
        return getLogTable().getSubTable(subsystem).getSubTable(deviceName);
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
        for (LogItem<?> item : items) {
            item.run();
        }
    }
}
