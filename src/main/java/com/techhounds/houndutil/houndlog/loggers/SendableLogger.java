package com.techhounds.houndutil.houndlog.loggers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.util.sendable.SendableBuilderImpl;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Basically a SingleItemLogger but it uses sendables. Useful for putting
 * anything Sendable like a Field2d, PIDController, or a motor controller.
 * 
 * @author dr
 * @apiNote do not put this or a LogGroup of this in a {@code periodic} method,
 *          since Sendables are sent to SmartDashboard declaratively. put these
 *          in an {@code init} method.
 */
public class SendableLogger extends Logger {
    private String key;
    private Sendable sendable;

    public SendableLogger(String subsystem, String key, Sendable sendable) {
        super(subsystem);
        this.key = key;
        this.sendable = sendable;
    }

    public SendableLogger(String key, Sendable sendable) {
        this.key = key;
        this.sendable = sendable;
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
        builder.setTable(getDataTable());
        SendableRegistry.publish(sendable, builder);
        builder.startListeners();
        getDataTable().getEntry(".name").setString(key);
    }

    /**
     * Gets the table associated with the subsystem + Sendable.
     */
    @Override
    protected NetworkTable getDataTable() {
        return getLogTable().getSubTable(subsystem).getSubTable(key);
    }

    /**
     * Since Sendables only need to be sent declaratively, all logic only needs to
     * be put in an init. {@link SendableRegistry} will handle everything else.
     */
    @Override
    public void init() {
        NetworkTable logTable = NetworkTableInstance.getDefault().getTable("HoundLog");
        publishSendable(logTable);
    }

    /**
     * Updates data for the object.
     */
    @Override
    public void run() {
        SendableRegistry.update(sendable);
    }
}
