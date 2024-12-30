package com.techhounds.houndutil.houndlog.loggers;

/**
 * Defines a group of {@link Loggable}s to log. This is useful for grouping
 * related logged items together, around a subsystem, device, sensor, or other
 * logical
 * grouping.
 * 
 * @author dr
 */
public class LogGroup implements Loggable {
    /**
     * The name of the log group. This key is used in the resulting table generated
     * from this group.
     */
    private String key;
    /** The LogGroup that contains this object. This can be null. */
    private LogGroup parent;
    /** The loggable objects that this group contains. */
    private Loggable[] loggers;

    /**
     * Creates a new LogGroup with the given key and loggers.
     * 
     * @param key     the name of the log group
     * @param loggers the loggable objects to include in this group
     */
    public LogGroup(String key, Loggable... loggers) {
        this.key = key;
        this.loggers = loggers;

        for (Loggable logger : loggers) {
            logger.setParent(this);
        }
    }

    /**
     * Calls the init method of all loggers in this group.
     */
    @Override
    public void init() {
        for (Loggable logger : loggers) {
            logger.init();
        }
    }

    /**
     * Calls the run method of all loggers in this group.
     */
    @Override
    public void run() {
        for (Loggable logger : loggers) {
            logger.run();
        }
    }

    @Override
    public String getFullPath() {
        if (parent != null) {
            return parent.getFullPath() + "/" + key;
        }
        return key;
    }

    @Override
    public void setParent(LogGroup parent) {
        this.parent = parent;
    }
}
