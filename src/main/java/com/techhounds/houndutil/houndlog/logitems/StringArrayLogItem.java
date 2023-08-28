package com.techhounds.houndutil.houndlog.logitems;

import java.util.function.Supplier;

import com.techhounds.houndutil.houndlog.enums.LogLevel;

import edu.wpi.first.networktables.StringArrayPublisher;

/**
 * The LogItem for string arrays.
 * 
 * @author dr
 */
public class StringArrayLogItem extends AbstractLogItem<String[]> {
    /** The publisher for this logger. */
    private StringArrayPublisher publisher;

    /**
     * Constructs a LogItem for String arrays.
     * 
     * @param subsystem     the subsystem to assign this LogItem to
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     * @param level         the level at which to place the LogItem
     */
    public StringArrayLogItem(String subsystem, String key, Supplier<String[]> func, LogLevel level) {
        super(subsystem, key, func, level);
    }

    /**
     * Constructs a LogItem for String arrays.
     * 
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     * @param level         the level at which to place the LogItem
     */
    public StringArrayLogItem(String key, Supplier<String[]> func, LogLevel level) {
        super(key, func, level);
    }

    /**
     * Constructs a LogItem for String arrays.
     * 
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     */
    public StringArrayLogItem(String key, Supplier<String[]> func) {
        super(key, func);
    }

    @Override
    public void publish() {
        publisher = getTable().getStringArrayTopic(key).publish();
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
