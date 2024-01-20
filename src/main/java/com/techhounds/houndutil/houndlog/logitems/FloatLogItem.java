package com.techhounds.houndutil.houndlog.logitems;

import java.util.function.Supplier;

import com.techhounds.houndutil.houndlog.enums.LogType;

import edu.wpi.first.networktables.FloatPublisher;
import edu.wpi.first.util.datalog.FloatLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * The LogItem for floats.
 * 
 * @author dr
 */
public class FloatLogItem extends AbstractLogItem<Float> {
    /** The publisher for this logger. */
    private FloatPublisher publisher;
    private FloatLogEntry datalogEntry;

    /**
     * Constructs a LogItem for floats.
     * 
     * @param subsystem     the subsystem to assign this LogItem to
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     * @param level         the level at which to place the LogItem
     */
    public FloatLogItem(String subsystem, String key, Supplier<Float> func, LogType level) {
        super(subsystem, key, func, level);
    }

    /**
     * Constructs a LogItem for floats.
     * 
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     * @param level         the level at which to place the LogItem
     */
    public FloatLogItem(String key, Supplier<Float> func, LogType level) {
        super(key, func, level);
    }

    /**
     * Constructs a LogItem for floats.
     * 
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     */
    public FloatLogItem(String key, Supplier<Float> func) {
        super(key, func);
    }

    @Override
    public void publish() {
        publisher = getTable().getFloatTopic(key).publish();
    }

    @Override
    public void unpublish() {
        publisher.close();
    }

    @Override
    public void createDatalogEntry() {
        datalogEntry = new FloatLogEntry(DataLogManager.getLog(), getFullName());
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
            float value = valueSupplier.get();
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
