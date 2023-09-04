package com.techhounds.houndutil.houndlog.logitems;

import java.util.Arrays;
import java.util.function.Supplier;

import com.techhounds.houndutil.houndlog.enums.LogType;

import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.util.datalog.IntegerArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * The LogItem for integer arrays.
 * 
 * @author dr
 */
public class IntegerArrayLogItem extends AbstractLogItem<int[]> {
    /** The publisher for this logger. */
    private IntegerArrayPublisher publisher;
    private IntegerArrayLogEntry datalogEntry;

    /**
     * Constructs a LogItem for integer arrays.
     * 
     * @param subsystem     the subsystem to assign this LogItem to
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     * @param level         the level at which to place the LogItem
     */
    public IntegerArrayLogItem(String subsystem, String key, Supplier<int[]> func, LogType level) {
        super(subsystem, key, func, level);
    }

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

    @Override
    public void publish() {
        publisher = getTable().getIntegerArrayTopic(key).publish();
    }

    @Override
    public void unpublish() {
        publisher.close();
    }

    @Override
    public void createDatalogEntry() {
        datalogEntry = new IntegerArrayLogEntry(DataLogManager.getLog(), getFullName());
    }

    @Override
    public void run() {
        if (this.type == LogType.NT) {
            if (publisher == null) {
                this.publish();
            }
            try {
                publisher.set(Arrays.stream(valueSupplier.get()).asLongStream().toArray());
            } catch (Exception e) {
                e.printStackTrace();
            }
        } else if (this.type == LogType.DATALOG) {
            if (datalogEntry == null) {
                this.createDatalogEntry();
            }

            int[] value = valueSupplier.get();
            if (this.previousValue == null || value != this.previousValue)
                datalogEntry.append(Arrays.stream(valueSupplier.get()).asLongStream().toArray());
            this.previousValue = value;
        }
    }
}
