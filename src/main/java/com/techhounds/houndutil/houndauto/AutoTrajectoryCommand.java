package com.techhounds.houndutil.houndauto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoTrajectoryCommand extends SequentialCommandGroup {
    protected AutoPath autoPath;

    public AutoTrajectoryCommand(AutoPath autoPath) {
        this.autoPath = autoPath;
    }

    public AutoTrajectoryCommand(AutoPath autoPath, Command... commands) {
        super(commands);
        this.autoPath = autoPath;
    }

    public AutoPath getAutoPath() {
        return autoPath;
    }
}
