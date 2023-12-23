package com.techhounds.houndutil.houndlog.logitems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.techhounds.houndutil.houndlog.enums.LogType;
import com.techhounds.houndutil.houndlog.loggers.Logger;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The base class for NT logging of primitive types. Can be used individually or
 * grouped together in a LogGroup or DeviceLogger.
 * 
 * @author dr
 */
public abstract class AbstractLogItem<T> extends Logger {
    /**
     * The name of the subsystem that this log item is logging under.
     * In NT, the hierarchy is defined as <subsystem>/<subkeys>/<key>.
     */
    protected String subsystem;

    /**
     * A list of subkeys, defining an arbitrary organization structure under the
     * subsystem.
     * In NT, the hierarchy is defined as <subsystem>/<subkeys>/<key>.
     */
    protected List<String> subkeys = new ArrayList<String>();

    /**
     * The key, or name, of this log item.
     * 
     * In NT, the hierarchy is defined as <subsystem>/<subkeys>/<key>.
     */
    protected String key;

    /**
     * The supplier of the value to be logged.
     */
    protected Supplier<T> valueSupplier;

    /**
     * The level at which to log this item at.
     */
    protected LogType type;

    protected T previousValue;

    /**
     * Constructs a {@code LogItem}.
     * 
     * @param subsystem     the subsystem to assign this LogItem to
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     * @param level         the level at which to place the LogItem
     */
    public AbstractLogItem(String subsystem, String key, Supplier<T> valueSupplier, LogType level) {
        this.subsystem = subsystem;
        this.key = key;
        this.valueSupplier = valueSupplier;
        this.type = level;
    }

    /**
     * Constructs a {@code LogItem}.
     * 
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     * @param level         the level at which to place the LogItem
     */
    public AbstractLogItem(String key, Supplier<T> valueSupplier, LogType level) {
        this("Not set", key, valueSupplier, level);
    }

    /**
     * Constructs a {@code LogItem}.
     * 
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     */
    public AbstractLogItem(String key, Supplier<T> valueSupplier) {
        this("Not set", key, valueSupplier, LogType.NT);
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
     * Get the level that this item is logging at.
     * 
     * @return the level
     */
    public LogType getType() {
        return type;
    }

    /**
     * Get the fully "drilled-down" table that this item is logging at, based on its
     * subsystem and subkeys.
     * 
     * @return the item's table
     */
    public NetworkTable getTable() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("HoundLog");
        if (!subsystem.equals(""))
            table = table.getSubTable(subsystem);

        for (String subkey : subkeys) {
            table = table.getSubTable(subkey);
        }
        return table;
    }

    public String getFullName() {
        ArrayList<String> keys = new ArrayList<String>();
        keys.add("NT"); // so that AdvantageScope displays everything together
        keys.add("HoundLog");
        if (!subsystem.equals(""))
            keys.add(subsystem);
        keys.addAll(subkeys);
        keys.add(key);
        return String.join("/", keys);
    }

    /**
     * Sets the subsystem of this log item.
     * 
     * @param subsystem the subsystem to set
     */
    public void setSubsystem(String subsystem) {
        this.subsystem = subsystem;
    }

    /**
     * Sets the subkeys for this log item.
     * 
     * @param subkeys the subkeys to set.
     */
    public void setSubkeys(List<String> subkeys) {
        this.subkeys = subkeys;
    }

    public T getValue() {
        return valueSupplier.get();
    }

    /**
     * Does nothing, since primitive type loggers do not need initialization.
     */
    public void init() {
    };

    /**
     * Sends the value through NT continuously if the LogItem should be actively
     * logging.
     */
    public abstract void run();

    /**
     * Publishes the key to NT.
     */
    public abstract void publish();

    /**
     * Unpublishes the key from NT when the item is no longer active.
     */
    public abstract void unpublish();

    public abstract void createDatalogEntry();
}
