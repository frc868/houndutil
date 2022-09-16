package com.techhounds.houndutil.houndlog.enums;

/**
 * The type of Logger to create. These are values defined by NetworkTables.
 * 
 * @author dr
 */
public enum LogLevel {
    /**
     * LogValues set at this level will only run when debug mode is on
     */
    DEBUG,
    /**
     * LogValues set at this level will only run when the bot is set in test mode
     * (does not have to enabled)
     */
    INFO,
    /**
     * LogValues set at this level will run all the time.
     */
    MAIN
}
