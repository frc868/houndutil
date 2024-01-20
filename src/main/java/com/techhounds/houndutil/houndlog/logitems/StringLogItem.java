package com.techhounds.houndutil.houndlog.logitems;

import java.util.function.Supplier;

import com.techhounds.houndutil.houndlog.enums.LogType;

import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * The LogItem for integers.
 * 
 * @author dr
 */
public class StringLogItem extends AbstractLogItem<String> {
    /** The publisher for this logger. */
    private StringPublisher publisher;
    private StringLogEntry datalogEntry;

    /**
     * Constructs a LogItem for integers.
     * 
     * @param subsystem     the subsystem to assign this LogItem to
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     * @param level         the level at which to place the LogItem
     */
    public StringLogItem(String subsystem, String key, Supplier<String> func, LogType level) {
        super(subsystem, key, func, level);
    }

    /**
     * Constructs a LogItem for integers.
     * 
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     * @param level         the level at which to place the LogItem
     */
    public StringLogItem(String key, Supplier<String> func, LogType level) {
        super(key, func, level);
    }

    /**
     * Constructs a LogItem for integers.
     * 
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     */
    public StringLogItem(String key, Supplier<String> func) {
        super(key, func);
    }

    @Override
    public void publish() {
        publisher = getTable().getStringTopic(key).publish();
    }

    @Override
    public void unpublish() {
        publisher.close();
    }

    @Override
    public void createDatalogEntry() {
        datalogEntry = new StringLogEntry(DataLogManager.getLog(), getFullName());
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
            String value = valueSupplier.get();
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
