package com.techhounds.houndutil.houndlog.logitems;

import java.util.function.DoubleConsumer;

import com.techhounds.houndutil.houndlog.enums.LogType;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj.DriverStation;

public class TunableDouble extends AbstractLogItem<Double> {
    private double initialValue;
    private double currentValue;
    private DoubleConsumer consumer;
    private DoubleEntry entry;

    private boolean needsReset = false;

    public TunableDouble(String key, double initialValue, DoubleConsumer consumer) {
        super(key, null, LogType.NT);
        this.initialValue = initialValue;
        currentValue = initialValue;

        this.consumer = consumer;
    }

    public TunableDouble(String subsystem, String key, double initialValue, DoubleConsumer consumer) {
        super(subsystem, key, null, LogType.NT);
        setSubsystem(subsystem);

        this.initialValue = initialValue;
        currentValue = initialValue;

        this.consumer = consumer;
    }

    public TunableDouble(String subsystem, String key, double initialValue) {
        this(subsystem, key, initialValue, (d) -> {
            System.err.println(key + " has not been given a consumer.");
        }); // empty consumer
    }

    @Override
    public void run() {
        if (entry == null)
            this.publish();
        if (DriverStation.isTest()) {
            double newValue = entry.get();
            if (newValue != currentValue) {
                consumer.accept(newValue);
                currentValue = newValue;
            }
            needsReset = true;
        } else {
            if (needsReset) {
                currentValue = initialValue;
                consumer.accept(initialValue);
                needsReset = false;
            }
            entry.set(currentValue);
        }
    }

    @Override
    public void publish() {
        entry = getTable().getDoubleTopic(key).getEntry(initialValue);
        entry.set(initialValue);
    }

    @Override
    public void unpublish() {
        // TunableNumbers always need to show up, so we don't need to have anything
        // here.
    }

    @Override
    public void createDatalogEntry() {
    }

    public double get() {
        run();
        return currentValue;
    }

    public void setConsumer(DoubleConsumer consumer) {
        this.consumer = consumer;
    }
}
