package com.techhounds.houndutil.houndlog.logitems;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Callable;

import com.techhounds.houndutil.houndlog.enums.LogLevel;
import com.techhounds.houndutil.houndlog.loggers.Logger;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public abstract class LogItem<T> extends Logger {
    protected String subsystem;
    protected List<String> subkeys = new ArrayList<String>();

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
    protected boolean isLogging;

    /**
     * Constructs a {@code LogItem}.
     * 
     * @param subsytem the subsystem to assign this LogItem to
     * @param key      the key of the value to log
     * @param func     the function to call to get the value
     * @param level    the level at which to place the LogItem
     */
    public LogItem(String subsystem, String key, Callable<T> func, LogLevel level) {
        this.subsystem = subsystem;
        this.key = key;
        this.func = func;
        this.level = level;
        isLogging = level.equals(LogLevel.MAIN);
    }

    /**
     * Constructs a {@code LogItem}.
     * 
     * @param key   the key of the value to log
     * @param func  the function to call to get the value
     * @param level the level at which to place the LogItem
     */
    public LogItem(String key, Callable<T> func, LogLevel level) {
        this("Not set", key, func, level);
    }

    /**
     * Constructs a {@code LogItem}.
     * 
     * @param key  the key of the value to log
     * @param func the function to call to get the value
     */
    public LogItem(String key, Callable<T> func) {
        this("Not set", key, func, LogLevel.INFO);
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
    public NetworkTable getTable() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("HoundLog").getSubTable(subsystem);
        for (String subkey : subkeys) {
            table = table.getSubTable(subkey);
        }
        return table;
    }

    public void setSubsystem(String subsystem) {
        this.subsystem = subsystem;
    }

    public void setSubkeys(List<String> subkeys) {
        this.subkeys = subkeys;
    }

    public void changeLevel(LogLevel newLevel, LogLevel oldLevel) {
        boolean previousIsLogging = this.isLogging;

        if (newLevel.getValue() >= this.level.getValue()) {
            isLogging = true;
        } else {
            isLogging = false;
        }

        if (previousIsLogging && !isLogging) {
            unpublish();
        } else if (!previousIsLogging && isLogging) {
            publish();
        }
    }

    public void init() {
    };

    public abstract void run();

    public abstract void publish();

    public abstract void unpublish();

}
