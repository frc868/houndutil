package com.techhounds.houndutil.houndlog.logitems;

import java.util.function.Supplier;

import com.techhounds.houndutil.houndlog.enums.LogType;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * The LogItem for booleans.
 * 
 * @author dr
 */
public class BooleanLogItem extends AbstractLogItem<Boolean> {
    /** The publisher for this logger. */
    private BooleanPublisher publisher;
    private BooleanLogEntry datalogEntry;

    /**
     * Constructs a LogItem for booleans.
     * 
     * @param subsystem     the subsystem to assign this LogItem to
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     * @param level         the level at which to place the LogItem
     */
    public BooleanLogItem(String subsystem, String key, Supplier<Boolean> func, LogType level) {
        super(subsystem, key, func, level);
    }

    /**
     * Constructs a LogItem for booleans.
     * 
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     * @param level         the level at which to place the LogItem
     */
    public BooleanLogItem(String key, Supplier<Boolean> func, LogType level) {
        super(key, func, level);
    }

    /**
     * Constructs a LogItem for booleans.
     * 
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     */
    public BooleanLogItem(String key, Supplier<Boolean> func) {
        super(key, func);
    }

    @Override
    public void publish() {
        publisher = getTable().getBooleanTopic(key).publish();
    }

    @Override
    public void unpublish() {
        publisher.close();
    }

    @Override
    public void createDatalogEntry() {
        datalogEntry = new BooleanLogEntry(DataLogManager.getLog(), getFullName());
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
            boolean value = valueSupplier.get();
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
