package com.techhounds.houndutil.houndlog.loggers;

/**
 * Base interface for any item that can be logged by HoundLog.
 * 
 * @author dr
 */
public interface Loggable {
    /**
     * Code to run on initialization of this Loggable.
     */
    public void init();

    /**
     * Code to run on every loop iteration for this Loggable.
     */
    public void run();

    /**
     * Set the parent LogGroup of this Loggable.
     */
    public void setParent(LogGroup parent);

    /**
     * Gets the full path of this Loggable. This path includes each parent
     * LogGroup's name in order, combined with the name of this Loggable.
     * 
     * @return the full path of this Loggable
     */
    public String getFullPath();
}
