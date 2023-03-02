package com.techhounds.houndutil.houndauto;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoTrajectoryCommand extends SequentialCommandGroup {
    private Optional<AutoPath> autoPath = Optional.empty();

    private Pose2d blueInitialPose;
    private Pose2d redInitialPose;

    public AutoTrajectoryCommand() {
        this.blueInitialPose = new Pose2d();
        this.redInitialPose = TrajectoryReflector.reflectiveTransformPose(blueInitialPose, 16.54);
    }

    public AutoTrajectoryCommand(AutoPath autoPath) {
        this.autoPath = Optional.of(autoPath);
        this.blueInitialPose = autoPath.getTrajectories().get(0).getInitialHolonomicPose();
        this.redInitialPose = TrajectoryReflector.reflectiveTransformPose(blueInitialPose, 16.54);
    }

    public AutoTrajectoryCommand(AutoPath autoPath, Command... commands) {
        super(commands);
        this.autoPath = Optional.of(autoPath);
        this.blueInitialPose = autoPath.getTrajectories().get(0).getInitialHolonomicPose();
        this.redInitialPose = TrajectoryReflector.reflectiveTransformPose(blueInitialPose, 16.54);
    }

    public AutoTrajectoryCommand(Pose2d blueInitialPose) {
        this.blueInitialPose = blueInitialPose;
        this.redInitialPose = TrajectoryReflector.reflectiveTransformPose(blueInitialPose, 16.54);
    }

    public AutoTrajectoryCommand(Pose2d blueInitialPose, Command... commands) {
        super(commands);
        this.blueInitialPose = blueInitialPose;
        this.redInitialPose = TrajectoryReflector.reflectiveTransformPose(blueInitialPose, 16.54);
    }

    public AutoTrajectoryCommand(Pose2d blueInitialPose, AutoPath autoPath, Command... commands) {
        super(commands);
        this.autoPath = Optional.of(autoPath);
        this.blueInitialPose = blueInitialPose;
        this.redInitialPose = TrajectoryReflector.reflectiveTransformPose(blueInitialPose, 16.54);
    }

    public Optional<AutoPath> getAutoPath() {
        return autoPath;
    }

    public Pose2d getInitialPosition() {
        return (DriverStation.getAlliance() == Alliance.Blue) ? blueInitialPose : redInitialPose;
    }
}
