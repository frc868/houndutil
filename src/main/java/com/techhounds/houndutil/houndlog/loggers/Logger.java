package com.techhounds.houndutil.houndlog.loggers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The base representation of a logger. Since this is abstract, it will not be
 * able to be instantiated and you must use a superclass.
 * 
 * @author dr
 */
public abstract class Logger implements Loggable {
    /**
     * The name of the subsystem to log under (the naming convention in SD is
     * SmartDashboard/{subsystem}/{device_name}/{value_name}).
     */
    protected String subsystem;

    /**
     * Instantiate a Logger.
     * 
     * @param subsystem the name of the subsystem
     */
    protected Logger(String subsystem) {
        this.subsystem = subsystem;
    }

    /**
     * Instantiate a logger (without a subsystem, only use if setting
     * the subsystem through LogGroup).
     */
    protected Logger() {
        this.subsystem = "Not set";
    }

    /**
     * Gets the name of the subsystem.
     * 
     * @return the name of the subsystem.
     */
    public String getSubsystem() {
        return subsystem;
    }

    /**
     * Set the name of the subsystem.
     * 
     * @param subsystem the name of the subsystem to set
     */
    public void setSubsystem(String subsystem) {
        this.subsystem = subsystem;
    }

    /**
     * Gets the base log table used by all {@link Logger} objects.
     */
    public NetworkTable getLogTable() {
        return NetworkTableInstance.getDefault().getTable("HoundLog");
    }

    /**
     * Gets the data table associated with this logger. This should be overridden,
     * so it has been made abstract. This can be either the base subsystem table or
     * the table for a specific device.
     * 
     * @return the target {@link NetworkTable}
     */
    protected abstract NetworkTable getDataTable();

    /**
     * Inits the logger. This is useful for things that need to be inited, like a
     * SendableLogger.
     */
    public abstract void init();

    /**
     * Runs the logger. This can really do anything, but should be used to run
     * {@code logItem()} on every item you want to log.
     */
    public abstract void run();
}
