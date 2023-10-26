package com.techhounds.houndutil.houndauto;

import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoRoutine {
    private String name;
    private List<AutoSetting> autoSettings;
    private Supplier<AutoPath> autoPathSupplier;
    private Supplier<Pose2d> blueInitialPoseSupplier;
    private Supplier<Pose2d> redInitialPoseSupplier;

    private Function<AutoPath, Command> commandGetter;

    /**
     * Initialize an AutoRoutine.
     * 
     * @param name            the name of the routine, this will be pushed to
     *                        Shuffleboard
     * @param commandSupplier the command to run, containing its trajectories
     */
    public AutoRoutine(String name, List<AutoSetting> autoSettings,
            Supplier<AutoPath> autoPathSupplier, Supplier<Pose2d> blueInitialPoseSupplier,
            Function<AutoPath, Command> commandGetter) {
        this.name = name;
        this.commandGetter = commandGetter;
        this.autoPathSupplier = autoPathSupplier;
        this.autoSettings = autoSettings;
        this.blueInitialPoseSupplier = blueInitialPoseSupplier;
        // this.redInitialPoseSupplier = () -> TrajectoryReflector.reflectiveTransformPose(blueInitialPoseSupplier.get(),
        //         16.54);
    }

    public String getName() {
        return name;
    }

    public List<AutoSetting> getAutoSettings() {
        return autoSettings;
    }

    public Optional<AutoPath> getAutoPath() {
        return autoPathSupplier != null ? Optional.of(autoPathSupplier.get()) : Optional.empty();
    }

    public Pose2d getInitialPosition() {
        // return (DriverStation.getAlliance() == Alliance.Blue) ? blueInitialPoseSupplier.get()
        //         : redInitialPoseSupplier.get();
        // TODO
        return blueInitialPoseSupplier.get();
    }

    public Command getCommand(AutoPath autoPath) {
        return commandGetter.apply(autoPath);
    }
}
