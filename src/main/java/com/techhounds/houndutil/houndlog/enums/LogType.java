package com.techhounds.houndutil.houndlog.enums;

/**
 * The type of Logger to create. These are values defined by NetworkTables.
 * 
 * @author dr
 */
public enum LogType {
    /**
     * LogValues set at this type will only be available via the console (over NT).
     */
    CONSOLE(2),
    /**
     * LogValues set at this level will be available via DataLog, but not NT.
     */
    DATALOG(1),
    /**
     * LogValues set at this level will be available over NT, always.
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
