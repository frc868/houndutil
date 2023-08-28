package com.techhounds.houndutil.houndlog.logitems;

import java.util.function.DoubleConsumer;

import com.techhounds.houndutil.houndlog.enums.LogLevel;

import edu.wpi.first.networktables.DoubleEntry;

public class TunableNumber extends AbstractLogItem<Double> {
    double value;
    DoubleConsumer consumer;
    DoubleEntry entry;

    public TunableNumber(String key, double initialValue, DoubleConsumer consumer) {
        super(key, null, LogLevel.MAIN);
        value = initialValue;

        this.consumer = consumer;
    }

    public TunableNumber(String subsystem, String key, double initialValue, DoubleConsumer consumer) {
        super(subsystem, key, null, LogLevel.MAIN);
        setSubsystem(subsystem + "/Tunables");
        value = initialValue;
        this.consumer = consumer;
    }

    public TunableNumber(String subsystem, String key, double initialValue) {
        this(subsystem, key, initialValue, (d) -> {
            System.err.println(key + " has not been given a consumer.");
        }); // empty consumer
    }

    @Override
    public void run() {
        if (entry == null)
            this.publish();
        double newValue = entry.get();
        if (newValue != value) {
            consumer.accept(newValue);
            value = newValue;
        }
    }

    @Override
    public void publish() {
        entry = getTable().getDoubleTopic(key).getEntry(value);
        entry.set(value);
    }

    @Override
    public void unpublish() {
        // TunableNumbers always need to show up, so we don't need to have anything
        // here.
    }

    public double get() {
        run();
        return value;
    }

    public void setConsumer(DoubleConsumer consumer) {
        this.consumer = consumer;
    }
}
