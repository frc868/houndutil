package com.techhounds.houndutil.houndlog;

/**
 * The type of Logger to create. This will determine where the log value is
 * visible. Certain loggers may need to be accessible at all times via a
 * NetworkTables connection, others may only want to be reviewed offline, and
 * others should only be visible in a debug console, when explicitly requested.
 * 
 * @author dr
 */
public enum LogType {
    /**
     * LogValues set at this type will only be available over HoundLog's console.
     * 
     * @deprecated This log type is currently unimplemented.
     */
    @Deprecated
    DEBUG(2),
    /**
     * LogValues set at this level will be available via data logs, but not NT.
     */
    DATALOG(1),
    /**
     * LogValues set at this level will be available over NetworkTables (but will
     * also be stored in data logs).
     */
    NT(0);

    private int value;

    LogType(int value) {
        this.value = value;
    }

    public int getValue() {
        return value;
    }
}
