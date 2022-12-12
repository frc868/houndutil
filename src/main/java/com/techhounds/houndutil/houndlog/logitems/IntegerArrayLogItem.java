package com.techhounds.houndutil.houndlog.logitems;

import java.util.Arrays;
import java.util.concurrent.Callable;

import com.techhounds.houndutil.houndlog.enums.LogLevel;

import edu.wpi.first.networktables.IntegerArrayPublisher;

/**
 * Defines a log item. This is using generics because the type of the log item
 * can change ({@code String}, {@code double}, {@code bool}, {@code Sendable},
 * etc).
 * 
 * @author dr
 */
public class IntegerArrayLogItem extends LogItem<int[]> {
    private IntegerArrayPublisher publisher;

    /**
     * Constructs a {@code LogItem}.
     * 
     * @param type  the type of log to create
     * @param key   the key of the value to log
     * @param func  the function to call to get the value
     * @param level the level at which to place the LogItem
     */
    public IntegerArrayLogItem(String key, Callable<int[]> func, LogLevel level) {
        super(key, func, level);
    }

    /**
     * Constructs a {@code LogItem}.
     * 
     * @param type the type of log to create
     * @param key  the key of the value to log
     * @param func the function to call to get the value
     */
    public IntegerArrayLogItem(String key, Callable<int[]> func) {
        this(key, func, LogLevel.INFO);
    }

    /**
     * Initializes the publisher. The data table is required to be set.
     */
    public void initPublisher() {
        if (getDataTable() == null) {
            System.err.println("The data table has to be set to initialize \"" + key + "\".");
        }

        try {
            publisher = getDataTable().getIntegerArrayTopic(key).publish();
        } catch (Exception e) {
            e.printStackTrace();
        }
        inited = true;
    }

    /**
     * Logs a specific item by calling the functions listed to log and convert the
     * results to the correct type.
     * 
     * Also checks the level of the LogItem.
     * 
     * @param item the {@link LogItem} to log
     */
    public void run() {
        if (!inited) {
            initPublisher();
        }
        if (super.getToRun()) {
            try {
                // FIXME weird thing about WPILib beta? int array publisher takes in a long
                // array
                publisher.set(Arrays.stream(func.call()).asLongStream().toArray());

            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }
}
