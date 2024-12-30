package com.techhounds.houndutil.houndlog.loggers;

import com.techhounds.houndutil.houndlog.LogType;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * An implementation of a LogItem that allows the user to change the value of a
 * specific boolean variable at runtime, or send that value to a consumer
 * function.
 * 
 * <p>
 * 
 * Tunable variables are only able to be changed when the Driver Station is set
 * to test mode.
 */
public class TunableBoolean extends LogItem<Boolean> {
    /** The initial value of the variable. */
    private boolean initialValue;
    /** The current value of the variable. */
    private boolean currentValue;
    /** The consumer function for the variable, if needed. */
    private BooleanConsumer consumer;
    /** The NetworkTables entry for the variable. */
    private BooleanEntry entry;

    /**
     * Flag indicating if the entry should be reset back to the initial value, when
     * the user switches out of test mode.
     */
    private boolean needsReset = false;

    /**
     * Constructs a TunableBoolean.
     * 
     * @param key          the key of the value to log
     * @param initialValue the initial value of the variable
     * @param consumer     the consumer function for the variable
     */
    public TunableBoolean(String key, boolean initialValue, BooleanConsumer consumer) {
        super(key, null, LogType.NT);
        this.initialValue = initialValue;
        currentValue = initialValue;

        this.consumer = consumer;
    }

    /**
     * Constructs a TunableBoolean.
     * 
     * @param key          the key of the value to log
     * @param initialValue the initial value of the variable
     */
    public TunableBoolean(String key, boolean initialValue) {
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

    /**
     * Publishes the key to NetworkTables.
     */
    @Override
    public void publish() {
        entry = getTable().getBooleanTopic(key).getEntry(initialValue);
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
    public boolean get() {
        run();
        return currentValue;
    }

    /**
     * Sets the the consumer function for the variable.
     * 
     * @param consumer the consumer function for the variable
     */
    public void setConsumer(BooleanConsumer consumer) {
        this.consumer = consumer;
    }
}
