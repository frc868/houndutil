package com.techhounds.houndutil.houndlib.subsystems.finished;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public abstract class FinishedSuperstructure extends FinishedSubsystemBase {
    private final String NAME = "Superstructure";
    //private final FinishedSwerveDrive[] drivetrain;
    private FinishedIntake[] feeders;
    private FinishedIntake[] hoppers;
    private FinishedIntake[] intakes;
    private FinishedFlywheel[] shooters;
    //private final FinishedVision[] vision;

    /**
     * Constructor for finished superstructure. 
     * <b>NOTE!</b> if certain subsystem(s) is not present, mark value as <i>'null'</i>. 
     * If marked as null, <b>DO NOT</b> call any methods on that subsystem, as it will throw a <i>null pointer exception</i> and your sim will break :(
    */
    public FinishedSuperstructure(FinishedIntake[] feeders, FinishedIntake[] hoppers, FinishedIntake[] intakes, FinishedFlywheel[] shooters) {
        this.feeders = feeders;
        this.hoppers = hoppers;
        this.intakes = intakes;
        this.shooters = shooters;
    }
        
    public Command feedCommand() {
        if (feeders != null && hoppers != null) {
            return Commands.parallel(
                feeders[0].runRollersCommand(),
                Commands.waitSeconds(0.1).andThen(hoppers[0].runRollersCommand())
            ).withName(NAME + ".feedCommand");
        } else {
            return Commands.none()
            .withName(NAME + ".rejectedFeedCommand");
        }
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
        if (intakes != null && hoppers != null) {
            return Commands.parallel(
                intakes[0].reverseRollersCommand().asProxy(),
                hoppers[0].reverseRollersCommand().asProxy()
            ).withName(NAME + ".feedReverseCommand");
        } else {
            return Commands.none().withName(NAME + ".rejectedFeedReverseCommand");
        }
    }

    /**
     * Creates a command that moves everything to zero. Useful before powering off.
     * 
     * @return the command
     */
    public abstract Command moveAllToZeroCommand();
}
