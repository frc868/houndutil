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
     * Gets the base log table used by all {@link Logger} objects.
     */
    public NetworkTable getBaseTable() {
        return NetworkTableInstance.getDefault().getTable("HoundLog");
    }

    /**
     * Runs the initialization sequence for the logger. This is run on robotInit,
     * and should be used for loggers that require initial setup.
     */
    public abstract void init();

    /**
     * Runs the logger. This should publish the value(s) to NT.
     */
    public abstract void run();

    /**
     * Sets the subsystem for this logger.
     * 
     * @param subsystem
     */
    public abstract void setSubsystem(String subsystem);
}
