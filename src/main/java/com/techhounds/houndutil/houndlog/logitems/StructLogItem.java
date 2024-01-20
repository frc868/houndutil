package com.techhounds.houndutil.houndlog.logitems;

import java.util.function.Supplier;

import com.techhounds.houndutil.houndlog.enums.LogType;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructTopic;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.DataLogManager;

public class StructLogItem<T> extends AbstractLogItem<T> {
    private StructPublisher<T> publisher;
    private StructLogEntry<T> datalogEntry;
    private Struct<T> struct;

    public StructLogItem(String subsystem, String key, Struct<T> struct, Supplier<T> func, LogType level) {
        super(subsystem, key, func, level);
        this.struct = struct;
    }

    public StructLogItem(String key, Struct<T> struct, Supplier<T> func, LogType level) {
        super(key, func, level);
        this.struct = struct;
    }

    public StructLogItem(String key, Struct<T> struct, Supplier<T> func) {
        super(key, func);
        this.struct = struct;
    }

    @Override
    public void publish() {
        StructTopic<T> topic = getTable().getStructTopic(key, struct);
        publisher = topic.publish();
    }

    @Override
    public void unpublish() {
        publisher.close();
    }

    @Override
    public void createDatalogEntry() {
        datalogEntry = StructLogEntry.create(DataLogManager.getLog(), getFullName(), struct);
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

/*
 * 
 */