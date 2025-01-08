package com.techhounds.houndutil.houndlog.loggers;

import java.util.function.DoubleConsumer;

import com.techhounds.houndutil.houndlog.LogType;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * An implementation of a LogItem that allows the user to change the value of a
 * specific double variable at runtime, or send that value to a consumer
 * function.
 * 
 * <p>
 * 
 * Tunable variables are only able to be changed when the Driver Station is set
 * to test mode.
 */
public class TunableDouble extends LogItem<Double> {
    /** The initial value of the variable. */
    private double initialValue;
    /** The current value of the variable. */
    private double currentValue;
    /** The consumer function for the variable, if needed. */
    private DoubleConsumer consumer;
    /** The NetworkTables entry for the variable. */
    private DoubleEntry entry;

    /**
     * Flag indicating if the entry should be reset back to the initial value, when
     * the user switches out of test mode.
     */
    private boolean needsReset = false;

    /**
     * Constructs a TunableDouble.
     * 
     * @param key          the key of the value to log
     * @param initialValue the initial value of the variable
     * @param consumer     the consumer function for the variable
     */
    public TunableDouble(String key, double initialValue, DoubleConsumer consumer) {
        super(key, null, LogType.NT);
        this.initialValue = initialValue;
        currentValue = initialValue;

        this.consumer = consumer;
    }

    /**
     * Constructs a TunableDouble.
     * 
     * @param key          the key of the value to log
     * @param initialValue the initial value of the variable
     */
    public TunableDouble(String key, double initialValue) {
        this(key, initialValue, (d) -> {
            System.err.println(key + " has not been given a consumer.");
        }); // empty consumer
    }

    /**
     * Gets the current value of the NetworkTables entry, if in test mode;
     * otherwise, sets the value of the NetworkTables entry to the initial value.
     */
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

    /**
     * Publishes the key to NetworkTables.
     */
    @Override
    public void publish() {
        entry = getTable().getDoubleTopic(key).getEntry(initialValue);
        entry.set(initialValue);
    }

    /** Does nothing, because Tunables always need to be published. */
    @Override
    public void unpublish() {
    }

    /** Does nothing, because tunable items never need to be in a data log. */
    @Override
    public void createDatalogEntry() {
    }

    /**
     * Gets the current value of the variable.
     * 
     * @return the current value of the variable
     */
    public double get() {
        run();
        return currentValue;
    }

    /**
     * Sets the the consumer function for the variable.
     * 
     * @param consumer the consumer function for the variable
     */
    public void setConsumer(DoubleConsumer consumer) {
        this.consumer = consumer;
    }
}
