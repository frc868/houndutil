package com.techhounds.houndutil.houndlog.loggers;

import java.util.function.Supplier;

import com.techhounds.houndutil.houndlog.LogType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The base class for logging an individual item. Can be used individually or as
 * part of a {@link LogGroup}.
 * 
 * @author dr
 */
/**
 * An abstract class representing a log item that can be logged using the
 * HoundLog system.
 * LogItem provides functionality for logging values to NetworkTables and
 * creating data log entries.
 *
 * @param <T> the type of value to be logged
 */
public abstract class LogItem<T> implements Loggable {
    /**
     * The name of this item. This is the final key that the item is referenced by.
     */
    protected String key;
    /** The LogGroup that contains this object. This can be null. */
    protected LogGroup parent;
    /**
     * The supplier of the value to be logged.
     */
    protected Supplier<T> valueSupplier;

    /**
     * The level at which to log this item at.
     */
    protected LogType type;

    /**
     * The previously fetched value from the {@code valueSupplier}, used to reduce
     * the size of data logs by only writing when a value changes.
     */
    protected T previousValue;

    /**
     * Constructs a LogItem.
     *
     * @param key           the name of the log item
     * @param valueSupplier the supplier of the value to be logged
     * @param level         the level at which to log this item
     */
    public LogItem(String key, Supplier<T> valueSupplier, LogType level) {
        this.key = key;
        this.valueSupplier = valueSupplier;
        this.type = level;
    }

    /**
     * Constructs a LogItem that logs to NetworkTables by default.
     *
     * @param key           the name of the log item
     * @param valueSupplier the supplier of the value to be logged
     */
    public LogItem(String key, Supplier<T> valueSupplier) {
        this(key, valueSupplier, LogType.NT);
    }

    /**
     * Initializes the log item. Does nothing, since primitive type loggers do not
     * need initialization.
     */
    @Override
    public void init() {
    };

    /**
     * Gets the value that should be logged, and handles it depending on the
     * {@link LogType}.
     * 
     * Subclasses must implement this method.
     */
    @Override
    public abstract void run();

    @Override
    public void setParent(LogGroup parent) {
        this.parent = parent;
    }

    @Override
    public String getFullPath() {
        if (parent != null) {
            return parent.getFullPath() + "/" + key;
        }
        return key;
    }

    /**
     * Gets the root table where this LogItem should be published.
     *
     * @return the root table for this LogItem
     */
    public NetworkTable getTable() {
        String tableName;
        if (parent != null) {
            tableName = parent.getFullPath();
        } else {
            tableName = "unassigned";
        }
        return NetworkTableInstance.getDefault().getTable(tableName);
    }

    /**
     * Publishes the key to NetworkTables.
     * Subclasses must implement this method.
     */
    public abstract void publish();

    /**
     * Unpublishes the key from NetworkTables when the item is no longer active.
     * Subclasses must implement this method.
     */
    public abstract void unpublish();

    /**
     * Creates a data log entry for this LogItem.
     * Subclasses must implement this method.
     */
    public abstract void createDatalogEntry();
}
