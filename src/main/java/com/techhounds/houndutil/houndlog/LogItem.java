package com.techhounds.houndutil.houndlog;

import java.util.concurrent.Callable;

import com.techhounds.houndutil.houndlog.enums.LogLevel;
import com.techhounds.houndutil.houndlog.enums.LogType;

/**
 * Defines a log item. This is using generics because the type of the log item
 * can change ({@code String}, {@code double}, {@code bool}, {@code Sendable},
 * etc).
 * 
 * @author dr
 */
public class LogItem<T> {
    /**
     * The type of log to create. This is a NetworkTables-defined convention.
     */
    private LogType type;
    /**
     * The key of the value you want to log. Basically, this is the name of the log
     * item.
     */
    private String key;
    /**
     * The function to call to get the value. This will be called at 50hz, so make
     * sure it's safe to use at that frequency. This must be of return type T.
     */
    private Callable<T> func;

    /**
     * The level at which to set this LogItem. This defines when this value is
     * updated to NetworkTables.
     */
    private LogLevel level;

    /**
     * Constructs a {@code LogItem}.
     * 
     * @param type the type of log to create
     * @param key  the key of the value to log
     * @param func the function to call to get the value
     */
    public LogItem(LogType type, String key, Callable<T> func) {
        this.type = type;
        this.key = key;
        this.func = func;
        this.level = LogLevel.INFO;
    }

    /**
     * Constructs a {@code LogItem}.
     * 
     * @param type  the type of log to create
     * @param key   the key of the value to log
     * @param func  the function to call to get the value
     * @param level the level at which to place the LogItem
     */
    public LogItem(LogType type, String key, Callable<T> func, LogLevel level) {
        this.type = type;
        this.key = key;
        this.func = func;
        this.level = level;
    }

    /**
     * Gets the type of the log item.
     * 
     * @return the log type
     */
    public LogType getType() {
        return type;
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
     * Gets the function associated with obtaining the log value.
     * 
     * @return the function associated with obtaining the log value
     */
    public Callable<T> getFunc() {
        return func;
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
}
