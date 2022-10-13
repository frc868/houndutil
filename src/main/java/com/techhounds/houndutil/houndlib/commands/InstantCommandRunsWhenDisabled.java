package com.techhounds.houndutil.houndlib.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class InstantCommandRunsWhenDisabled extends InstantCommand {
    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
