package com.techhounds.houndutil.houndlib.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class FinishedSuperstructure extends SubsystemBase {
    private final String name = "Superstructure";
    private final FinishedIntake intake;

    /**Constructor for finished superstructure. 
     * *NOTE!* if certain subsystem(s) is not present, mark value as 'null' 
    */
    public FinishedSuperstructure(FinishedIntake intake) {
        this.intake = intake;
    }

    /**
     * Creates a command that waits to run the hopper until the feeder has spun
     * up.
     * 
     * @return the command
     */
    public abstract Command feedCommand();

    /**
     * Creates a command that feeds into the shooter only while the robot is ready
     * to shoot, stopping and resuming as necessary.
     * 
     * @return the command
     */
    public abstract Command feedWhileReadyCommand();

    /**
     * Creates a command that runs the indexing systems in reverse to eject Fuel out
     * the intake.
     * 
     * @return the command
     */
    public Command feedReverseCommand() {
        return Commands.parallel(
            intake.reverseRollersCommand().asProxy()
            //hopper.reverseRollersCommand.asProxy()
        ).withName(name + ".feedReverseCommand");
    }

    /**
     * Creates a command that moves everything to zero. Useful before powering off.
     * 
     * @return the command
     */
    public abstract Command moveAllToZeroCommand();
}
