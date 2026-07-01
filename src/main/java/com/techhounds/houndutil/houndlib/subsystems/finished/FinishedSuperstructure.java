package com.techhounds.houndutil.houndlib.subsystems.finished;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class FinishedSuperstructure extends FinishedSubsystemBase {
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
                allFeedersRunRollers(),
                Commands.waitSeconds(0.1).andThen(allHoppersRunRollers())
            ).withName(NAME + ".feedCommand");
        } else {
            return Commands.none()
            .withName(NAME + ".rejectedFeedCommand");
        }
    }

    /**
     * Creates a command that runs the indexing systems in reverse to eject Fuel out
     * the intake.
     * 
     * @return the command
     */
    public Command feedReverseCommand() {
        if (intakes != null && hoppers != null) {
            return Commands.parallel(
                allIntakesReverseRollers(),
                allHoppersReverseRollers()
            ).withName(NAME + ".feedReverseCommand");
        } else {
            return Commands.none().withName(NAME + ".rejectedFeedReverseCommand");
        }
    }

    public Command allFeedersRunRollers() {
        if (feeders == null || feeders.length == 0) {
            return Commands.none().withName(NAME + ".rejectedAllFeedersRunRollers");
        }

        Command[] feederCommands = new Command[feeders.length];
        for (int i = 0; i < feeders.length; i++) {
            feederCommands[i] = feeders[i].runRollersCommand().asProxy();
        }

        return Commands.parallel(feederCommands).withName(NAME + ".allFeedersRunRollers");
    }

    public Command allFeedersReverseRollers() {
        if (feeders == null || feeders.length == 0) {
            return Commands.none().withName(NAME + ".rejectedAllFeedersReverseRollers");
        }

        Command[] feederCommands = new Command[feeders.length];
        for (int i = 0; i < feeders.length; i++) {
            feederCommands[i] = feeders[i].reverseRollersCommand().asProxy();
        }

        return Commands.parallel(feederCommands).withName(NAME + ".allFeedersReverseRollers");
    }

    public Command allHoppersRunRollers() {
        if (hoppers == null || hoppers.length == 0) {
            return Commands.none().withName(NAME + ".rejectedAllHoppersRunRollers");
        }

        Command[] hopperCommands = new Command[hoppers.length];
        for (int i = 0; i < hoppers.length; i++) {
            hopperCommands[i] = hoppers[i].runRollersCommand().asProxy();
        }

        return Commands.parallel(hopperCommands).withName(NAME + ".allHoppersRunRollers");
    }

    public Command allHoppersReverseRollers() {
        if (hoppers == null || hoppers.length == 0) {
            return Commands.none().withName(NAME + ".rejectedAllHoppersReverseRollers");
        }

        Command[] hopperCommands = new Command[hoppers.length];
        for (int i = 0; i < hoppers.length; i++) {
            hopperCommands[i] = hoppers[i].reverseRollersCommand().asProxy();
        }

        return Commands.parallel(hopperCommands).withName(NAME + ".allHoppersRunRollers");
    }

    public Command allIntakesRunRollers() {
        if (intakes == null || intakes.length == 0) {
            return Commands.none().withName(NAME + ".rejectedAllIntakesRunRollers");
        }

        Command[] intakeCommands = new Command[intakes.length];
        for (int i = 0; i < hoppers.length; i++) {
            intakeCommands[i] = intakes[i].runRollersCommand().asProxy();
        }

        return Commands.parallel(intakeCommands).withName(NAME + ".allIntakesRunRollers");
    }

    public Command allIntakesReverseRollers() {
        if (intakes == null || intakes.length == 0) {
            return Commands.none().withName(NAME + ".rejectedAllIntakesReverseRollers");
        }

        Command[] intakeCommands = new Command[intakes.length];
        for (int i = 0; i < intakes.length; i++) {
            intakeCommands[i] = intakes[i].runRollersCommand().asProxy();
        }

        return Commands.parallel(intakeCommands).withName(NAME + ".allIntakesReverseRollers");
    }
}
