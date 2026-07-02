package com.techhounds.houndutil.houndlib.subsystems.finished;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class FinishedSuperstructure<Intake extends FinishedIntake, Shooter extends FinishedFlywheel> {
    //private final FinishedSwerveDrive[] drivetrain;
    private Intake[] feeders;
    private Intake[] hoppers;
    private Intake[] intakes;
    private Shooter[] shooters;
    //private final FinishedVision[] vision;

    /**
     * Constructor for finished superstructure. 
     * <b>NOTE!</b> if certain subsystem(s) is not present, mark value as <i>'null'</i>. 
     * If marked as null, <b>DO NOT</b> call any methods on that subsystem, as it will throw a <i>null pointer exception</i> and your sim will break :(
    */
    public FinishedSuperstructure(Intake[] feeders, Intake[] hoppers, Intake[] intake, Shooter[] shooter) {
        this.feeders = feeders;
        this.hoppers = hoppers;
        this.intakes = intake;
        this.shooters = shooter;
    }
        
    public Command feedCommand(Voltage feederVoltage, Voltage hopperVoltage) {
        if (feeders != null && hoppers != null) {
            return Commands.parallel(
                allFeedersRunRollers(feederVoltage),
                Commands.waitSeconds(0.1).andThen(allHoppersRunRollers(hopperVoltage))
            ).withName("Superstructure.feedCommand");
        } else {
            return Commands.none()
            .withName("Superstructure.rejectedFeedCommand");
        }
    }

    /**
     * Creates a command that runs the indexing systems in reverse to eject Fuel out
     * the intake.
     * 
     * @return the command
     */
    public Command feedReverseCommand(Voltage feederVoltage, Voltage hopperVoltage) {
        if (intakes != null && hoppers != null) {
            return Commands.parallel(
                allIntakesReverseRollers(feederVoltage),
                allHoppersReverseRollers(hopperVoltage)
            ).withName("Superstructure.feedReverseCommand");
        } else {
            return Commands.none().withName("Superstructure.rejectedFeedReverseCommand");
        }
    }

    public Command allFeedersRunRollers(Voltage voltage) {
        if (feeders == null || feeders.length == 0) {
            return Commands.none().withName("Superstructure.rejectedAllFeedersRunRollers");
        }

        Command[] feederCommands = new Command[feeders.length];
        for (int i = 0; i < feeders.length; i++) {
            feederCommands[i] = feeders[i].runRollersCommand(voltage).asProxy();
        }

        return Commands.parallel(feederCommands).withName("Superstructure.allFeedersRunRollers");
    }

    public Command allFeedersReverseRollers(Voltage voltage) {
        if (feeders == null || feeders.length == 0) {
            return Commands.none().withName("Superstructure.rejectedAllFeedersReverseRollers");
        }

        Command[] feederCommands = new Command[feeders.length];
        for (int i = 0; i < feeders.length; i++) {
            feederCommands[i] = feeders[i].runRollersCommand(voltage).asProxy();
        }

        return Commands.parallel(feederCommands).withName("Superstructure.allFeedersReverseRollers");
    }

    public Command allHoppersRunRollers(Voltage voltage) {
        if (hoppers == null || hoppers.length == 0) {
            return Commands.none().withName("Superstructure.rejectedAllHoppersRunRollers");
        }

        Command[] hopperCommands = new Command[hoppers.length];
        for (int i = 0; i < hoppers.length; i++) {
            hopperCommands[i] = hoppers[i].runRollersCommand(voltage).asProxy();
        }

        return Commands.parallel(hopperCommands).withName("Superstructure.allHoppersRunRollers");
    }

    public Command allHoppersReverseRollers(Voltage voltage) {
        if (hoppers == null || hoppers.length == 0) {
            return Commands.none().withName("Superstructure.rejectedAllHoppersReverseRollers");
        }

        Command[] hopperCommands = new Command[hoppers.length];
        for (int i = 0; i < hoppers.length; i++) {
            hopperCommands[i] = hoppers[i].runRollersCommand(voltage).asProxy();
        }

        return Commands.parallel(hopperCommands).withName("Superstructure.allHoppersRunRollers");
    }

    public Command allIntakesRunRollers(Voltage voltage) {
        if (intakes == null || intakes.length == 0) {
            return Commands.none().withName("Superstructure.rejectedAllIntakesRunRollers");
        }

        Command[] intakeCommands = new Command[intakes.length];
        for (int i = 0; i < hoppers.length; i++) {
            intakeCommands[i] = intakes[i].runRollersCommand(voltage).asProxy();
        }

        return Commands.parallel(intakeCommands).withName("Superstructure.allIntakesRunRollers");
    }

    public Command allIntakesReverseRollers(Voltage voltage) {
        if (intakes == null || intakes.length == 0) {
            return Commands.none().withName("Superstructure.rejectedAllIntakesReverseRollers");
        }

        Command[] intakeCommands = new Command[intakes.length];
        for (int i = 0; i < intakes.length; i++) {
            intakeCommands[i] = intakes[i].runRollersCommand(voltage).asProxy();
        }

        return Commands.parallel(intakeCommands).withName("Superstructure.allIntakesReverseRollers");
    }
}
