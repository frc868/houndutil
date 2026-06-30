package com.techhounds.houndutil.houndlib.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public abstract class FinishedSuperstructure extends FinishedSubsystemBase {
    private final String NAME = "Superstructure";
    //private final FinishedSwerveDrive drivetrain;
    private final FinishedIntake feeder;
    private final FinishedIntake hopper;
    private final FinishedIntake intake;
    private final FinishedFlywheel shooter;
    //private final FinishedVision vision;

    /**Constructor for finished superstructure. 
     * *NOTE!* if certain subsystem(s) is not present, mark value as 'null' 
    */
    public FinishedSuperstructure(FinishedSubsystemBase[] subsystems) {
        //drivetrain = (FinishedSwerveDrive) subsystems[0];
        feeder = (FinishedIntake) subsystems[1];
        hopper = (FinishedIntake) subsystems[2];
        intake = (FinishedIntake) subsystems[3];
        shooter = (FinishedFlywheel) subsystems[4];
        //vision = (FinishedVision) subsystems[5];
    }

    /**
     * Creates a command that waits to run the hopper until the feeder has spun
     * up.
     * 
     * @return the command
     */
    public Command feedCommand() {
        return Commands.parallel(
            feeder.runRollersCommand(),
            Commands.waitSeconds(0.1).andThen(hopper.runRollersCommand())
        ).withName(NAME + ".feedCommand");
    }

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
            intake.reverseRollersCommand().asProxy(),
            hopper.reverseRollersCommand().asProxy()
        ).withName(NAME + ".feedReverseCommand");
    }

    /**
     * Creates a command that moves everything to zero. Useful before powering off.
     * 
     * @return the command
     */
    public abstract Command moveAllToZeroCommand();
}
