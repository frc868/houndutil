package com.techhounds.houndutil.houndlog.loggers;

import java.util.Arrays;
import java.util.function.Supplier;

import com.techhounds.houndutil.houndlog.LogType;

import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.util.datalog.IntegerArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * LogItem for integer arrays.
 */
public class IntegerArrayLogItem extends LogItem<int[]> {
    /** The NetworkTables publisher for this logger. */
    private IntegerArrayPublisher publisher;
    /** The data log publisher for this logger. */
    private IntegerArrayLogEntry datalogEntry;

    /**
     * Constructs a LogItem for integer arrays.
     * 
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     * @param level         the level at which to place the LogItem
     */
    public IntegerArrayLogItem(String key, Supplier<int[]> func, LogType level) {
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

    /**
     * Publishes the key to NetworkTables.
     */
    @Override
    public void publish() {
        publisher = getTable().getIntegerArrayTopic(key).publish();
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
        datalogEntry = new IntegerArrayLogEntry(DataLogManager.getLog(), getFullPath());
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
            int[] value = valueSupplier.get();
            if (this.previousValue == null || value != this.previousValue) {
                if (this.type == LogType.NT)
                    publisher.set(Arrays.stream(valueSupplier.get()).asLongStream().toArray());
                else if (this.type == LogType.DATALOG)
                    datalogEntry.append(Arrays.stream(valueSupplier.get()).asLongStream().toArray());
            }
            this.previousValue = value;
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
