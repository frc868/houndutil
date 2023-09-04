package com.techhounds.houndutil.houndlog;

import com.techhounds.houndutil.houndlog.loggers.Loggable;
import com.techhounds.houndutil.houndlog.loggers.Logger;

/**
 * Defines a group of {@link Logger}s to log. This is useful when used
 * in
 * a subsystem where you only want to call one {@code logger.run()} method.
 * 
 * @author dr
 */
public class LogGroup implements Loggable {
    private Logger[] loggers;

    /**
     * Instantiates a new LogGroup. Gets the subsystem name from the first Logger in
     * the array.
     * 
     * @param loggers the loggers to assign to this LogGroup
     */
    public LogGroup(Logger... loggers) {
        this.loggers = loggers;
    }

    /**
     * Instantiates a new LogGroup. Sets the subsystem name in all Loggers.
     * 
     * @param subsystem
     * @param loggers
     */
    public LogGroup(String subsystem, Logger... loggers) {
        this.loggers = loggers;
        setLoggerSubsystems(subsystem);
    }

    /**
     * Sets the subsystems of all loggers in the group.
     * 
     * @param subsystem the name of the subsystem to set.
     */
    public void setLoggerSubsystems(String subsystem) {
        for (Logger logger : loggers) {
            logger.setSubsystem(subsystem);
        }
    }

    @Override
    public void init() {
        for (Logger logger : loggers) {
            logger.init();
        }
    }

    @Override
    public void run() {
        for (Logger logger : loggers) {
            logger.run();
        }
    }
}
