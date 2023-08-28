package com.techhounds.houndutil.houndlog.logitems;

import java.util.Arrays;
import java.util.function.Supplier;

import com.techhounds.houndutil.houndlog.enums.LogLevel;

import edu.wpi.first.networktables.IntegerArrayPublisher;

/**
 * The LogItem for integer arrays.
 * 
 * @author dr
 */
public class IntegerArrayLogItem extends AbstractLogItem<int[]> {
    /** The publisher for this logger. */
    private IntegerArrayPublisher publisher;

    /**
     * Constructs a LogItem for integer arrays.
     * 
     * @param subsystem     the subsystem to assign this LogItem to
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     * @param level         the level at which to place the LogItem
     */
    public IntegerArrayLogItem(String subsystem, String key, Supplier<int[]> func, LogLevel level) {
        super(subsystem, key, func, level);
    }

    /**
     * Constructs a LogItem for integer arrays.
     * 
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     * @param level         the level at which to place the LogItem
     */
    public IntegerArrayLogItem(String key, Supplier<int[]> func, LogLevel level) {
        super(key, func, level);
    }

    /**
     * Constructs a LogItem for integer arrays.
     * 
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     */
    public IntegerArrayLogItem(String key, Supplier<int[]> func) {
        super(key, func);
    }

    @Override
    public void publish() {
        publisher = getTable().getIntegerArrayTopic(key).publish();
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
                publisher.set(Arrays.stream(valueSupplier.get()).asLongStream().toArray());
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }
}
