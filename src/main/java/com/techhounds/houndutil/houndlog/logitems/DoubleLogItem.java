package com.techhounds.houndutil.houndlog.logitems;

import java.util.function.Supplier;

import com.techhounds.houndutil.houndlog.enums.LogLevel;

import edu.wpi.first.networktables.DoublePublisher;

/**
 * The LogItem for doubles.
 * 
 * @author dr
 */
public class DoubleLogItem extends AbstractLogItem<Double> {
    /** The publisher for this logger. */
    private DoublePublisher publisher;

    /**
     * Constructs a LogItem for doubles.
     * 
     * @param subsystem     the subsystem to assign this LogItem to
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     * @param level         the level at which to place the LogItem
     */
    public DoubleLogItem(String subsystem, String key, Supplier<Double> valueSupplier, LogLevel level) {
        super(subsystem, key, valueSupplier, level);
    }

    /**
     * Constructs a LogItem for doubles.
     * 
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     * @param level         the level at which to place the LogItem
     */
    public DoubleLogItem(String key, Supplier<Double> valueSupplier, LogLevel level) {
        super(key, valueSupplier, level);
    }

    /**
     * Constructs a LogItem for doubles.
     * 
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     */
    public DoubleLogItem(String key, Supplier<Double> valueSupplier) {
        super(key, valueSupplier);
    }

    @Override
    public void publish() {
        publisher = getTable().getDoubleTopic(key).publish();
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
