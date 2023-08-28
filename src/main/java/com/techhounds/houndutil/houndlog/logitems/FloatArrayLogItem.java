package com.techhounds.houndutil.houndlog.logitems;

import java.util.function.Supplier;

import com.techhounds.houndutil.houndlog.enums.LogLevel;

import edu.wpi.first.networktables.FloatArrayPublisher;

/**
 * The LogItem for float arrays.
 * 
 * @author dr
 */
public class FloatArrayLogItem extends AbstractLogItem<float[]> {
    /** The publisher for this logger. */
    private FloatArrayPublisher publisher;

    /**
     * Constructs a LogItem for float arrays.
     * 
     * @param subsystem     the subsystem to assign this LogItem to
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     * @param level         the level at which to place the LogItem
     */
    public FloatArrayLogItem(String subsystem, String key, Supplier<float[]> func, LogLevel level) {
        super(subsystem, key, func, level);
    }

    /**
     * Constructs a LogItem for float arrays.
     * 
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     * @param level         the level at which to place the LogItem
     */
    public FloatArrayLogItem(String key, Supplier<float[]> func, LogLevel level) {
        super(key, func, level);
    }

    /**
     * Constructs a LogItem for float arrays.
     * 
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     */
    public FloatArrayLogItem(String key, Supplier<float[]> func) {
        super(key, func);
    }

    @Override
    public void publish() {
        publisher = getTable().getFloatArrayTopic(key).publish();
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
