package com.techhounds.houndutil.houndlog;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;
import com.techhounds.houndutil.houndlib.robots.HoundRobot;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;
import com.techhounds.houndutil.houndlog.loggers.LogGroup;
import com.techhounds.houndutil.houndlog.loggers.LogItem;
import com.techhounds.houndutil.houndlog.loggers.Loggable;

import edu.wpi.first.wpilibj.Timer;

/**
 * Manages all registered loggers, and handles generating loggers from
 * annotations given a base class.
 * 
 * <p>
 * 
 * Call {@code LoggingManager.getInstance().registerRobotContainer(this)} in the
 * {@code RobotContainer} constructor. If not using {@link HoundRobot}, call
 * {@code LoggingManager.getInstance().init()} in {@code robotInit()} and
 * {@code LoggingManager.getInstance().run()} in {@code robotPeriodic()}.
 */
@LoggedObject
public class LoggingManager {
    /** The singleton instance of the logging manager. */
    private static LoggingManager instance;
    // this log group won't contain anything, but exists so it can be the parent of
    // all log groups
    private LogGroup baseLogGroup = new LogGroup("HoundLog");
    private List<Loggable> loggables = new ArrayList<Loggable>();
    private Map<Class<?>, Function<Object, LogItem<?>[]>> profiles = new HashMap<Class<?>, Function<Object, LogItem<?>[]>>();

    private static double startTime = Timer.getFPGATimestamp();
    @Log
    private static double loggingLoopTimeMs = 0.0;

    private LoggingManager() {
        registerProfiles(LogProfiles.class);
    }

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
     * Registers an object to search for log annotations.
     * 
     * @param object the object to register
     */
    public void registerObject(Object object) {
        try {
            LogAnnotationHandler.handleLoggedObject(object);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * Registers a class to search for log annotations. Used for classes with
     * <b>static</b> fields and methods.
     * 
     * @param clazz the subsystems to register
     */
    public void registerClass(Class<?> clazz, String name, ArrayList<String> subkeys) {
        try {
            LogAnnotationHandler.handleLoggedClass(clazz, name, subkeys);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * Registers a class to search for log profiles. These are methods that take in
     * a single object and return an array of LogItems, and are annotated with
     * {@code @LogProfile}.
     * 
     * @param clazz the class to register
     */
    public void registerProfiles(Class<?> clazz) {
        LogAnnotationHandler.handleLogProfile(clazz, profiles);
    }

    /**
     * Adds a group to the registered list of loggers.
     * 
     * @param group the LogGroup to add
     */
    public void addGroup(LogGroup group) {
        loggables.add(group);
        group.setParent(baseLogGroup);
    }

    /**
     * Adds an individual Loggable to the registered list of loggers.
     * 
     * @param loggable the Loggable to add
     */
    public void addLogger(Loggable loggable) {
        loggables.add(loggable);
        loggable.setParent(baseLogGroup);
    }

    /**
     * Gets all of the {@link Loggable}s currently registered.
     * 
     * @return the registered loggables
     */
    public List<Loggable> getLoggables() {
        return loggables;
    }

    /**
     * Gets the registered log profiles.
     * 
     * @return the registered log profiles
     */
    protected Map<Class<?>, Function<Object, LogItem<?>[]>> getProfiles() {
        return profiles;
    }

    /**
     * Runs the {@code init()} method on each loggable. Call this in
     * {@code robotInit()}.
     */
    public void init() {
        for (Loggable loggable : loggables) {
            loggable.init();
        }
    }

    /**
     * Runs the {@code run()} method on each loggable. Call this in
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
