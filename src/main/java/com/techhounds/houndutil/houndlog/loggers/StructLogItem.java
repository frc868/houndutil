package com.techhounds.houndutil.houndlog.loggers;

import java.util.function.Supplier;

import com.techhounds.houndutil.houndlog.LogType;

import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructTopic;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * LogItem for objects that contain a set of fields defined by a {@code Struct}.
 */
public class StructLogItem<T> extends LogItem<T> {
    /** The NetworkTables publisher for this logger. */
    private StructPublisher<T> publisher;
    /** The data log publisher for this logger. */
    private StructLogEntry<T> datalogEntry;
    /** The Struct that defines the fields of the object. */
    private Struct<T> struct;

    /**
     * Constructs a LogItem for objects that contain a set of fields defined by a
     * {@code Struct}.
     * 
     * @param key           the key of the value to log
     * @param struct        the Struct that defines the fields of the object
     * @param valueSupplier the supplier for the value
     * @param level         the level at which to place the LogItem
     */
    public StructLogItem(String key, Struct<T> struct, Supplier<T> func, LogType level) {
        super(key, func, level);
        this.struct = struct;
    }

    /**
     * Constructs a LogItem for objects that contain a set of fields defined by a
     * {@code Struct}.
     * 
     * @param key           the key of the value to log
     * @param struct        the Struct that defines the fields of the object
     * @param valueSupplier the supplier for the value
     */
    public StructLogItem(String key, Struct<T> struct, Supplier<T> func) {
        super(key, func);
        this.struct = struct;
    }

    /**
     * Publishes the key to NetworkTables.
     */
    @Override
    public void publish() {
        StructTopic<T> topic = getTable().getStructTopic(key, struct);
        publisher = topic.publish();
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
        datalogEntry = StructLogEntry.create(DataLogManager.getLog(), getFullPath(), struct);
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
            T value = valueSupplier.get();
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