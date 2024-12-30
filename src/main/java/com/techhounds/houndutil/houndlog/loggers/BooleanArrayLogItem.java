package com.techhounds.houndutil.houndlog.loggers;

import java.util.function.Supplier;

import com.techhounds.houndutil.houndlog.LogType;

import edu.wpi.first.networktables.BooleanArrayPublisher;
import edu.wpi.first.util.datalog.BooleanArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * LogItem for boolean arrays.
 */
public class BooleanArrayLogItem extends LogItem<boolean[]> {
    /** The NetworkTables publisher for this logger. */
    private BooleanArrayPublisher publisher;
    /** The data log publisher for this logger. */
    private BooleanArrayLogEntry datalogEntry;

    /**
     * Constructs a LogItem for boolean arrays.
     * 
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     * @param level         the level at which to place the LogItem
     */
    public BooleanArrayLogItem(String key, Supplier<boolean[]> func, LogType level) {
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

    /**
     * Publishes the key to NetworkTables.
     */
    @Override
    public void publish() {
        publisher = getTable().getBooleanArrayTopic(key).publish();
    }

    /**
     * Unpublishes the key from NetworkTables if the item no longer needs to be
     * active.
     */
    @Override
    public void unpublish() {
        publisher.close();
    }

    /**
     * Creates a data log entry for the key.
     */
    @Override
    public void createDatalogEntry() {
        datalogEntry = new BooleanArrayLogEntry(DataLogManager.getLog(), getFullPath());
    }

    /**
     * Gets the value that should be logged, and handles it depending on the
     * {@link LogType}.
     */
    @Override
    public void run() {
        if (this.type == LogType.NT) {
            if (publisher == null) {
                this.publish();
            }
        } else if (this.type == LogType.DATALOG) {
            if (datalogEntry == null) {
                this.createDatalogEntry();
            }

        }

        try {
            boolean[] value = valueSupplier.get();
            if (this.previousValue == null || value != this.previousValue) {
                if (this.type == LogType.NT)
                    publisher.set(value);
                else if (this.type == LogType.DATALOG)
                    datalogEntry.append(value);
            }
            this.previousValue = value;
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
