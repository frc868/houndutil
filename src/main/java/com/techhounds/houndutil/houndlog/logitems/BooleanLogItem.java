package com.techhounds.houndutil.houndlog.logitems;

import java.util.function.Supplier;

import com.techhounds.houndutil.houndlog.enums.LogLevel;

import edu.wpi.first.networktables.BooleanPublisher;

/**
 * The LogItem for booleans.
 * 
 * @author dr
 */
public class BooleanLogItem extends AbstractLogItem<Boolean> {
    /** The publisher for this logger. */
    private BooleanPublisher publisher;

    /**
     * Constructs a LogItem for booleans.
     * 
     * @param subsystem     the subsystem to assign this LogItem to
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     * @param level         the level at which to place the LogItem
     */
    public BooleanLogItem(String subsystem, String key, Supplier<Boolean> func, LogLevel level) {
        super(subsystem, key, func, level);
    }

    /**
     * Constructs a LogItem for booleans.
     * 
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     * @param level         the level at which to place the LogItem
     */
    public BooleanLogItem(String key, Supplier<Boolean> func, LogLevel level) {
        super(key, func, level);
    }

    /**
     * Constructs a LogItem for booleans.
     * 
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     */
    public BooleanLogItem(String key, Supplier<Boolean> func) {
        super(key, func);
    }

    @Override
    public void publish() {
        publisher = getTable().getBooleanTopic(key).publish();
    }

    @Override
    public void unpublish() {
        publisher.close();
    }

    public void run() {
        if (isLogging) {
            if (publisher == null) {
                this.publish();
            }
            try {
                publisher.set(valueSupplier.get());
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }
}
