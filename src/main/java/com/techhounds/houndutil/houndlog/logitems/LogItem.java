package com.techhounds.houndutil.houndlog.logitems;

import java.util.concurrent.Callable;

import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.enums.LogLevel;
import com.techhounds.houndutil.houndlog.loggers.Logger;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;

public abstract class LogItem<T> extends Logger {
    /**
     * The key of the value you want to log. Basically, this is the name of the log
     * item.
     */
    protected String key;

    /**
     * The function to call to get the value. This will be called at 50hz, so make
     * sure it's safe to use at that frequency. This must be of return type T.
     */
    protected Callable<T> func;

    /**
     * The level at which to set this LogItem. This defines when this value is
     * updated to NetworkTables.
     */
    protected LogLevel level;

    protected NetworkTable table;

    protected boolean inited = false;

    /**
     * Constructs a {@code LogItem}.
     * 
     * @param type  the type of log to create
     * @param key   the key of the value to log
     * @param func  the function to call to get the value
     * @param level the level at which to place the LogItem
     */
    public LogItem(String key, Callable<T> func, LogLevel level) {
        this.key = key;
        this.func = func;
        this.level = level;
    }

    /**
     * Constructs a {@code LogItem}.
     * 
     * @param type the type of log to create
     * @param key  the key of the value to log
     * @param func the function to call to get the value
     */
    public LogItem(String key, Callable<T> func) {
        this(key, func, LogLevel.INFO);
    }

    /**
     * Get the key of the log item.
     * 
     * @return the log key
     */
    public String getKey() {
        return key;
    }

    /**
     * Gets the log level. This determines when the value is logged and when it's
     * set aside for performance reasons.
     * 
     * @return the log level.
     */
    public LogLevel getLevel() {
        return level;
    }

    /**
     * Gets the log level. This determines when the value is logged and when it's
     * set aside for performance reasons.
     * 
     * @return the log level.
     */
    public NetworkTable getDataTable() {
        return table;
    }

    /**
     * Gets the log level. This determines when the value is logged and when it's
     * set aside for performance reasons.
     * 
     * @return the log level.
     */
    public void setDataTable(NetworkTable table) {
        this.table = table;
    }

    public boolean getToRun() {
        boolean run = false;
        switch (level) {
            case DEBUG:
                run = LoggingManager.getDebugMode();
                break;
            case INFO:
                run = DriverStation.isTest();
                break;
            case MAIN:
                run = true;
                break;
        }
        return run;
    }

    public void init() {
    };

    public void run() {
    };
}
