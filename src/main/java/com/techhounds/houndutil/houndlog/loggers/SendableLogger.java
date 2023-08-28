package com.techhounds.houndutil.houndlog.loggers;

import com.techhounds.houndutil.houndlog.enums.LogLevel;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Basically a SingleItemLogger but it uses sendables. Useful for putting
 * anything Sendable like a Field2d, PIDController, or a motor controller.
 * 
 * @author dr
 * @apiNote this class no longer has functionality in the 2023 version of
 *          WPILib. a replacement will be made soon.
 * 
 */
public class SendableLogger extends Logger {
    private String subsystem;
    private String key;
    private Sendable sendable;

    public SendableLogger(String subsystem, String key, Sendable sendable) {
        this.subsystem = subsystem;
        this.key = key;
        this.sendable = sendable;
    }

    public SendableLogger(String key, Sendable sendable) {
        this("Not set", key, sendable);
    }

    /**
     * This is required because I'm stubborn and don't want to use SmartDashboard
     * for this, so I had to copy the code to send a Sendable over NetworkTables
     * from {@link SmartDashboard}.
     *
     * @param logTable the table through which to send it
     */
    private void publishSendable(NetworkTable logTable) {
        SendableBuilderImpl builder = new SendableBuilderImpl();
        builder.setTable(logTable);
        SendableRegistry.publish(sendable, builder);
        builder.startListeners();
        logTable.getStringTopic(".name").publish().set(key);
    }

    /**
     * Gets the table associated with the subsystem + Sendable.
     */
    protected NetworkTable getTable() {
        return getBaseTable().getSubTable(subsystem).getSubTable(key);
    }

    @Override
    public void setSubsystem(String subsystem) {
        this.subsystem = subsystem;
    }

    /**
     * Since Sendables only need to be sent declaratively, all logic only needs to
     * be put in an init. {@link SendableRegistry} will handle everything else.
     */
    @Override
    public void init() {
        publishSendable(getTable());
    }

    /**
     * Updates data for the object.
     */
    @Override
    public void run() {
        SendableRegistry.update(sendable);
    }

    @Override
    public void handleLevelChange(LogLevel newLevel, LogLevel oldLevel) {

    }
}
