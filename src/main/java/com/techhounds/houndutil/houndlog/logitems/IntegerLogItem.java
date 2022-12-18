package com.techhounds.houndutil.houndlog.logitems;

import java.util.concurrent.Callable;

import com.techhounds.houndutil.houndlog.enums.LogLevel;

import edu.wpi.first.networktables.IntegerPublisher;

/**
 * Defines a log item. This is using generics because the type of the log item
 * can change ({@code String}, {@code double}, {@code bool}, {@code Sendable},
 * etc).
 * 
 * @author dr
 */
public class IntegerLogItem extends LogItem<Integer> {
    private IntegerPublisher publisher;

    /**
     * Constructs a {@code LogItem}.
     * 
     * @param type  the type of log to create
     * @param key   the key of the value to log
     * @param func  the function to call to get the value
     * @param level the level at which to place the LogItem
     */
    public IntegerLogItem(String key, Callable<Integer> func, LogLevel level) {
        super(key, func, level);
    }

    /**
     * Constructs a {@code LogItem}.
     * 
     * @param type the type of log to create
     * @param key  the key of the value to log
     * @param func the function to call to get the value
     */
    public IntegerLogItem(String key, Callable<Integer> func) {
        this(key, func, LogLevel.INFO);
    }

    /**
     * Initializes the publisher. The data table is required to be set.
     */
    @Override
    public void publish() {
        if (getTable() == null) {
            System.err.println("The data table has to be set to initialize \"" + key + "\".");
            return;
        }

        publisher = getTable().getIntegerTopic(key).publish();
    }

    @Override
    public void unpublish() {
        publisher.close();
    }

    /**
     * Logs a specific item by calling the functions listed to log and convert the
     * results to the correct type.
     * 
     * Also checks the level of the LogItem.
     * 
     * @param item the {@link LogItem} to log
     */
    @Override
    public void run() {
        if (isLogging) {
            if (publisher == null) {
                this.publish();
            }
            try {
                // Same as IntegerArrayLogItem, but Java automatically casts from int to long.
                publisher.set(func.call());
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }
}
