package com.techhounds.houndutil.houndlib.auto;

import edu.wpi.first.wpilibj2.command.Command;

public interface AutoTrajectoryCommand extends Command {
    public AutoPath getAutoPath();
}
