package com.techhounds.houndutil.houndlog.logitems;

import java.util.function.Supplier;

import com.techhounds.houndutil.houndlog.enums.LogLevel;

import edu.wpi.first.networktables.FloatPublisher;

/**
 * The LogItem for floats.
 * 
 * @author dr
 */
public class FloatLogItem extends AbstractLogItem<Float> {
    /** The publisher for this logger. */
    private FloatPublisher publisher;

    /**
     * Constructs a LogItem for floats.
     * 
     * @param subsystem     the subsystem to assign this LogItem to
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     * @param level         the level at which to place the LogItem
     */
    public FloatLogItem(String subsystem, String key, Supplier<Float> func, LogLevel level) {
        super(subsystem, key, func, level);
    }

    /**
     * Constructs a LogItem for floats.
     * 
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     * @param level         the level at which to place the LogItem
     */
    public FloatLogItem(String key, Supplier<Float> func, LogLevel level) {
        super(key, func, level);
    }

    /**
     * Constructs a LogItem for floats.
     * 
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     */
    public FloatLogItem(String key, Supplier<Float> func) {
        super(key, func);
    }

    @Override
    public void publish() {
        publisher = getTable().getFloatTopic(key).publish();
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
