package com.techhounds.houndutil.houndlog.logitems;

import java.util.function.Supplier;

import com.techhounds.houndutil.houndlog.enums.LogLevel;

import edu.wpi.first.networktables.DoubleArrayPublisher;

/**
 * The LogItem for double arrays.
 * 
 * @author dr
 */
public class DoubleArrayLogItem extends AbstractLogItem<double[]> {
    /** The publisher for this logger. */
    private DoubleArrayPublisher publisher;

    /**
     * Constructs a LogItem for double arrays.
     * 
     * @param subsystem     the subsystem to assign this LogItem to
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     * @param level         the level at which to place the LogItem
     */
    public DoubleArrayLogItem(String subsystem, String key, Supplier<double[]> func, LogLevel level) {
        super(subsystem, key, func, level);
    }

    /**
     * Constructs a LogItem for double arrays.
     * 
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     * @param level         the level at which to place the LogItem
     */
    public DoubleArrayLogItem(String key, Supplier<double[]> func, LogLevel level) {
        super(key, func, level);
    }

    /**
     * Constructs a LogItem for double arrays.
     * 
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     */
    public DoubleArrayLogItem(String key, Supplier<double[]> func) {
        super(key, func);
    }

    @Override
    public void publish() {
        publisher = getTable().getDoubleArrayTopic(key).publish();
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
