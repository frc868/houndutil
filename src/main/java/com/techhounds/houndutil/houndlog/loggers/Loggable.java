package com.techhounds.houndutil.houndlog.loggers;

import com.techhounds.houndutil.houndlog.enums.LogLevel;

/**
 * Defines a loggable. This was mainly necessary due to wanting to include both
 * {@link Logger}s and {@link LogGroup}s in {@link LoggingManager}.
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

    public void changeLevel(LogLevel newLevel, LogLevel oldLevel);
}
