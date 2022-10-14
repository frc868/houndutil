package com.techhounds.houndutil.houndlib.auto;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;

public interface AutoTrajectoryCommand extends Command {
    public ArrayList<AutoPath> getAutoPaths();
}
