package com.techhounds.houndutil.houndlog.logitems;

import java.util.function.Supplier;

import com.techhounds.houndutil.houndlog.enums.LogType;

import edu.wpi.first.networktables.BooleanArrayPublisher;
import edu.wpi.first.util.datalog.BooleanArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * The LogItem for boolean arrays.
 * 
 * @author dr
 */
public class BooleanArrayLogItem extends AbstractLogItem<boolean[]> {
    /** The publisher for this logger. */
    private BooleanArrayPublisher publisher;
    private BooleanArrayLogEntry datalogEntry;

    /**
     * Constructs a LogItem for boolean arrays.
     * 
     * @param subsystem     the subsystem to assign this LogItem to
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     * @param level         the level at which to place the LogItem
     */
    public BooleanArrayLogItem(String subsystem, String key, Supplier<boolean[]> func, LogType level) {
        super(subsystem, key, func, level);
    }

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

    @Override
    public void publish() {
        publisher = getTable().getBooleanArrayTopic(key).publish();
    }

    @Override
    public void unpublish() {
        publisher.close();
    }

    @Override
    public void createDatalogEntry() {
        datalogEntry = new BooleanArrayLogEntry(DataLogManager.getLog(), getFullName());
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
