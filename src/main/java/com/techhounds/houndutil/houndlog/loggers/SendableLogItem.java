package com.techhounds.houndutil.houndlog.loggers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;

/**
 * A logger for individual Sendable objects. Useful for interactable items like
 * the CommandScheduler, individual Commands, individual Subsystems, or an
 * AddressableLEDBuffer.
 */
public class SendableLogItem implements Loggable {
    /**
     * The name of this item. This is the final key that the item is referenced by.
     */
    private String key;
    /** The LogGroup that contains this object. This can be null. */
    private LogGroup parent;
    /** The Sendable object to be logged. */
    private Sendable sendable;

    /**
     * Constructs a SendableLogItem.
     *
     * @param key      the name of the log item
     * @param sendable the Sendable object to be logged
     */
    public SendableLogItem(String key, Sendable sendable) {
        this.key = key;
        this.sendable = sendable;
    }

    /**
     * Publishes the Sendable object to NetworkTables and adds it to the
     * SendableRegistry.
     * 
     * @param logTable the NetworkTable to publish the Sendable to
     */
    private void publishSendable(NetworkTable logTable) {
        SendableBuilderImpl builder = new SendableBuilderImpl();
        builder.setTable(logTable);
        SendableRegistry.publish(sendable, builder);
        builder.startListeners();
        logTable.getEntry(".name").setString(key);
    }

    /**
     * Since Sendables only need to be sent declaratively, all logic only needs to
     * be put in an init. {@link SendableRegistry} will handle everything else.
     */
    @Override
    public void init() {
        publishSendable(getTable().getSubTable(key));
    }

    /**
     * Updates the object in the SendableRegistry.
     */
    @Override
    public void run() {
        SendableRegistry.update(sendable);
    }

    @Override
    public void setParent(LogGroup parent) {
        this.parent = parent;
    }

    @Override
    public String getFullPath() {
        if (parent != null) {
            return parent.getFullPath() + "/" + key;
        }
        return key;
    }

    /**
     * Gets the root table where this LogItem should be published.
     *
     * @return the root table for this LogItem
     */
    public NetworkTable getTable() {
        String tableName;
        if (parent != null) {
            tableName = parent.getFullPath();
        } else {
            tableName = "unassigned";
        }
        return NetworkTableInstance.getDefault().getTable(tableName);
    }
}
