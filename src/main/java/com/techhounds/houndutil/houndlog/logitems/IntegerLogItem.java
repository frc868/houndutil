package com.techhounds.houndutil.houndlog.logitems;

import java.util.function.Supplier;

import com.techhounds.houndutil.houndlog.enums.LogLevel;

import edu.wpi.first.networktables.IntegerPublisher;

/**
 * The LogItem for integers.
 * 
 * @author dr
 */
public class IntegerLogItem extends AbstractLogItem<Integer> {
    /** The publisher for this logger. */
    private IntegerPublisher publisher;

    /**
     * Constructs a LogItem for integers.
     * 
     * @param subsystem     the subsystem to assign this LogItem to
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     * @param level         the level at which to place the LogItem
     */
    public IntegerLogItem(String subsystem, String key, Supplier<Integer> func, LogLevel level) {
        super(subsystem, key, func, level);
    }

    /**
     * Constructs a LogItem for integers.
     * 
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     * @param level         the level at which to place the LogItem
     */
    public IntegerLogItem(String key, Supplier<Integer> func, LogLevel level) {
        super(key, func, level);
    }

    /**
     * Constructs a LogItem for integers.
     * 
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     */
    public IntegerLogItem(String key, Supplier<Integer> func) {
        super(key, func);
    }

    @Override
    public void publish() {
        publisher = getTable().getIntegerTopic(key).publish();
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
