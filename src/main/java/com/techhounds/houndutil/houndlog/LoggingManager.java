package com.techhounds.houndutil.houndlog;

import java.util.ArrayList;
import java.util.List;
import com.techhounds.houndutil.houndlog.interfaces.Log;
import com.techhounds.houndutil.houndlog.interfaces.LoggedObject;
import com.techhounds.houndutil.houndlog.loggers.Loggable;
import com.techhounds.houndutil.houndlog.loggers.Logger;

import edu.wpi.first.wpilibj.Timer;

/**
 * A singleton manager for logging to avoid some of the pitfalls with using the
 * periodic methods of each subsystem (like the inability to use Test mode,
 * verbosity, etc).
 * 
 * @author dr
 */
@LoggedObject
public class LoggingManager {
    private static LoggingManager instance;
    private List<Loggable> loggables = new ArrayList<Loggable>();

    private static double startTime = Timer.getFPGATimestamp();
    @Log
    private static double loggingLoopTimeMs = 0.0;

    /**
     * Returns a singleton of LoggingManager.
     * 
     * @return a singleton LoggingManager.
     */
    public static LoggingManager getInstance() {
        if (instance == null) {
            instance = new LoggingManager();
        }
        return instance;
    }

    /**
     * Register subsystems to check for log annotations.
     * 
     * @param subsystems the subsystems to register
     */
    public void registerRobotContainer(Object robotContainer) {
        try {
            LogAnnotationHandler.handleLoggedObject(robotContainer);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * Register subsystems to check for log annotations.
     * 
     * @param subsystems the subsystems to register
     */
    public void registerClass(Class<?> class_, String name, ArrayList<String> subkeys) {
        try {
            LogAnnotationHandler.handleLoggedClass(class_, name, subkeys);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * Adds a group to the LoggingManager. Sets the subsystem of the group and its
     * loggers as well.
     * 
     * @param subsystem the name of the subsystem
     * @param group     the LogGroup to add
     */
    public void addGroup(String subsystem, LogGroup group) {
        group.setLoggerSubsystems(subsystem);
        loggables.add(group);
    }

    /**
     * Adds a group to the LoggingManager. Only use this constructor if the
     * subsystem is already defined in the LogGroup.
     * 
     * @param group the LogGroup to add
     */
    public void addGroup(LogGroup group) {
        loggables.add(group);
    }

    /**
     * Adds an individual Logger to the LoggingManager. Sets the subsystem of the
     * Logger as well.
     * 
     * @param logger the Logger to add
     */
    public void addLogger(String subsystem, Logger logger) {
        logger.setSubsystem(subsystem);
        loggables.add(logger);
    }

    /**
     * Adds an individual logger to the LoggingManager. Only use this constructor if
     * the subsystem is already defined in the Logger.
     * 
     * @param logger the Logger to add
     */
    public void addLogger(Logger logger) {
        loggables.add(logger);
    }

    /**
     * Get the {@link Loggable}s in the LoggingManager.
     * 
     * @return the loggables
     */
    public List<Loggable> getLoggables() {
        return loggables;
    }

    /**
     * Runs the {@code init()} method on each loggable. Put this in
     * {@code robotInit()}.
     */
    public void init() {
        for (Loggable loggable : loggables) {
            loggable.init();
        }
    }

    /**
     * Runs the {@code run()} method on each loggable. Put this in
     * {@code robotPeriodic()}.
     */
    public void run() {
        startTime = Timer.getFPGATimestamp();
        for (Loggable loggable : loggables) {
            loggable.run();
        }
        loggingLoopTimeMs = (Timer.getFPGATimestamp() - startTime) * 1000;
    }
}
