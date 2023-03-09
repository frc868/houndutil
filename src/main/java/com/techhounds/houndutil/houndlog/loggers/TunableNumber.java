package com.techhounds.houndutil.houndlog.loggers;

import java.util.function.DoubleConsumer;

import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.enums.LogLevel;
import com.techhounds.houndutil.houndlog.logitems.LogItem;

import edu.wpi.first.networktables.DoubleEntry;

public class TunableNumber extends LogItem<Double> {
    double value;
    DoubleConsumer consumer;
    DoubleEntry entry;

    public TunableNumber(String subsystem, String key, double initialValue, DoubleConsumer consumer) {
        super(key, null, LogLevel.MAIN);
        value = initialValue;
        setSubsystem(subsystem + "/Tunables");

        this.consumer = consumer;
        entry = getTable().getDoubleTopic(key).getEntry(value); // sets the default to value
        entry.set(value);
        LoggingManager.getInstance().addLogger(this); // adds itself to the manager so everything works
    }

    public TunableNumber(String subsystem, String key, double initialValue) {
        this(subsystem, key, initialValue, (d) -> {
            System.err.println(key + " has not been given a consumer.");
        }); // empty consumer
    }

    @Override
    public void run() {
        double newValue = entry.get();
        if (newValue != value) {
            consumer.accept(newValue);
            value = newValue;
        }

    }

    @Override
    public void publish() {
        // TunableNumbers always need to show up, so we don't need to have anything
        // here.
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
