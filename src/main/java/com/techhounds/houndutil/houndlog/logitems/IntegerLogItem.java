package com.techhounds.houndutil.houndlog.logitems;

import java.util.function.Supplier;

import com.techhounds.houndutil.houndlog.enums.LogType;

import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * The LogItem for integers.
 * 
 * @author dr
 */
public class IntegerLogItem extends AbstractLogItem<Integer> {
    /** The publisher for this logger. */
    private IntegerPublisher publisher;
    private IntegerLogEntry datalogEntry;

    /**
     * Constructs a LogItem for integers.
     * 
     * @param subsystem     the subsystem to assign this LogItem to
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     * @param level         the level at which to place the LogItem
     */
    public IntegerLogItem(String subsystem, String key, Supplier<Integer> func, LogType level) {
        super(subsystem, key, func, level);
    }

    /**
     * Constructs a LogItem for integers.
     * 
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     * @param level         the level at which to place the LogItem
     */
    public IntegerLogItem(String key, Supplier<Integer> func, LogType level) {
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

    @Override
    public void createDatalogEntry() {
        datalogEntry = new IntegerLogEntry(DataLogManager.getLog(), getFullName());
    }

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
            int value = valueSupplier.get();
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
