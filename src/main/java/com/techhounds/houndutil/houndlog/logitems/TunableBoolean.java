package com.techhounds.houndutil.houndlog.logitems;

import com.techhounds.houndutil.houndlog.enums.LogType;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.DriverStation;

public class TunableBoolean extends AbstractLogItem<Double> {
    private boolean initialValue;
    private boolean currentValue;
    private BooleanConsumer consumer;
    private BooleanEntry entry;

    private boolean needsReset = false;

    public TunableBoolean(String key, boolean initialValue, BooleanConsumer consumer) {
        super(key, null, LogType.NT);
        this.initialValue = initialValue;
        currentValue = initialValue;

        this.consumer = consumer;
    }

    public TunableBoolean(String subsystem, String key, boolean initialValue, BooleanConsumer consumer) {
        super(subsystem, key, null, LogType.NT);
        setSubsystem(subsystem);

        this.initialValue = initialValue;
        currentValue = initialValue;

        this.consumer = consumer;
    }

    public TunableBoolean(String subsystem, String key, boolean initialValue) {
        this(subsystem, key, initialValue, (d) -> {
            System.err.println(key + " has not been given a consumer.");
        }); // empty consumer
    }

    @Override
    public void run() {
        if (entry == null)
            this.publish();
        if (DriverStation.isTest()) {
            boolean newValue = entry.get();
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
        entry = getTable().getBooleanTopic(key).getEntry(initialValue);
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

    public boolean get() {
        run();
        return currentValue;
    }

    public void setConsumer(BooleanConsumer consumer) {
        this.consumer = consumer;
    }
}
