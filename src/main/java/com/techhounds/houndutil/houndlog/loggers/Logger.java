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
     * Inits the logger. This is useful for things that need to be inited, like a
     * SendableLogger.
     */
    public abstract void init();

    /**
     * Runs the logger. This can really do anything, but should be used to run
     * {@code logItem()} on every item you want to log.
     */
    public abstract void run();

    public abstract void setSubsystem(String subsystem);
}
