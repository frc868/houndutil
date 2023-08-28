package com.techhounds.houndutil.houndlog.logitems;

import java.util.function.Supplier;

import com.techhounds.houndutil.houndlog.enums.LogLevel;

import edu.wpi.first.networktables.BooleanArrayPublisher;

/**
 * The LogItem for boolean arrays.
 * 
 * @author dr
 */
public class BooleanArrayLogItem extends AbstractLogItem<boolean[]> {
    /** The publisher for this logger. */
    private BooleanArrayPublisher publisher;

    /**
     * Constructs a LogItem for boolean arrays.
     * 
     * @param subsystem     the subsystem to assign this LogItem to
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     * @param level         the level at which to place the LogItem
     */
    public BooleanArrayLogItem(String subsystem, String key, Supplier<boolean[]> func, LogLevel level) {
        super(subsystem, key, func, level);
    }

    /**
     * Constructs a LogItem for boolean arrays.
     * 
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     * @param level         the level at which to place the LogItem
     */
    public BooleanArrayLogItem(String key, Supplier<boolean[]> func, LogLevel level) {
        super(key, func, level);
    }

    /**
     * Constructs a LogItem for boolean arrays.
     * 
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     */
    public BooleanArrayLogItem(String key, Supplier<boolean[]> func) {
        super(key, func);
    }

    @Override
    public void publish() {
        publisher = getTable().getBooleanArrayTopic(key).publish();
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
