package com.techhounds.houndutil.houndlog.loggers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * LogItem for strings.
 */
public class MetadataLogger implements Loggable {
    private final MetadataRecord record;

    public record MetadataRecord(
            String mavenName,
            String gitSha,
            String gitDate,
            String gitBranch,
            String buildDate,
            int dirty) {
    }

    /**
     * Constructs a LogItem for integers.
     * 
     * @param key           the key of the value to log
     * @param valueSupplier the supplier for the value
     */
    public MetadataLogger(MetadataRecord record) {
        this.record = record;
    }

    @Override
    public void setParent(LogGroup parent) {
    }

    @Override
    public String getFullPath() {
        return "Metadata";
    }

    /**
     * Gets the root table where this LogItem should be published.
     *
     * @return the root table for this LogItem
     */
    public NetworkTable getTable() {
        return NetworkTableInstance.getDefault().getTable(getFullPath());
    }

    @Override
    public void init() {
        getTable().getStringTopic("projectName").publish().set(record.mavenName);
        getTable().getStringTopic("runtimeType").publish().set(RobotBase.getRuntimeType().toString());
        getTable().getStringTopic("gitSha").publish().set(record.gitSha);
        getTable().getStringTopic("gitDate").publish().set(record.gitDate);
        getTable().getStringTopic("gitBranch").publish().set(record.gitBranch);
        getTable().getStringTopic("gitDirty").publish()
                .set(record.dirty == 1 ? "Uncommitted changes" : "All changes committed");
        getTable().getStringTopic("buildDate").publish().set(record.buildDate);
    }

    @Override
    public void run() {
    }
}
