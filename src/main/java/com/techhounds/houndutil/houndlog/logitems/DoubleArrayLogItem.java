package com.techhounds.houndutil.houndlog.logitems;

import java.util.function.Supplier;

import com.techhounds.houndutil.houndlog.enums.LogType;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * The LogItem for double arrays.
 * 
 * @author dr
 */
public class DoubleArrayLogItem extends AbstractLogItem<double[]> {
    /** The publisher for this logger. */
    private DoubleArrayPublisher publisher;
    private DoubleArrayLogEntry datalogEntry;

    /**
     * Constructs a LogItem for double arrays.
     * 
     * @param subsystem     the subsystem to assign this LogItem to
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     * @param level         the level at which to place the LogItem
     */
    public DoubleArrayLogItem(String subsystem, String key, Supplier<double[]> func, LogType level) {
        super(subsystem, key, func, level);
    }

    /**
     * Constructs a LogItem for double arrays.
     * 
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     * @param level         the level at which to place the LogItem
     */
    public DoubleArrayLogItem(String key, Supplier<double[]> func, LogType level) {
        super(key, func, level);
    }

    /**
     * Constructs a LogItem for double arrays.
     * 
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     */
    public DoubleArrayLogItem(String key, Supplier<double[]> func) {
        super(key, func);
    }

    @Override
    public void publish() {
        publisher = getTable().getDoubleArrayTopic(key).publish();
    }

    @Override
    public void unpublish() {
        publisher.close();
    }

    @Override
    public void createDatalogEntry() {
        datalogEntry = new DoubleArrayLogEntry(DataLogManager.getLog(), getFullName());
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
            double[] value = valueSupplier.get();
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
