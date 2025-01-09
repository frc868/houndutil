package com.techhounds.houndutil.houndauto;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Consumer;
import java.util.stream.Collectors;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.techhounds.houndutil.houndlog.LogType;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.loggers.DoubleLogItem;
import com.techhounds.houndutil.houndlog.loggers.LogGroup;
import com.techhounds.houndutil.houndlog.loggers.SendableLogItem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Handles scheduling of autos, odometry resetting, and display of paths to
 * NetworkTables. Provides an interface for a drive team to select an autonomous
 * routine, and preview the path that will be followed along with the robot's
 * estimated starting position, given any localization.
 * 
 * Uses the {@code /HoundLog/autonomous} table.
 * 
 * <p>
 * 
 * Use {@code init()} in {@code robotInit},
 * {@code periodicUpdate()} in {@code disabledPeriodic},
 * {@code runSelectedRoutine()} in {@code autonomousInit}, and
 * {@code endRoutine()} in {@code autonomousExit}.
 * 
 * <p>
 * 
 * Note: this is automatically handled by {@code HoundRobot}.
 */
public class AutoManager {
    private static AutoManager instance;
    private HashMap<String, AutoRoutine> routines = new HashMap<String, AutoRoutine>();
    private AutoRoutine lastRoutine;
    private SendableChooser<AutoRoutine> chooser = new SendableChooser<AutoRoutine>();
    private Field2d field = new Field2d();
    /**
     * The scheduled command without the additional composition (extra odometry
     * triggers, timer sets).
     */
    private Command baseCommand;
    /** The full scheduled command, used for exiting the routine. */
    private Command currentCommand;
    /** A consumer that takes in a Pose2d to reset the odometry of a drivetrain. */
    private Consumer<Pose2d> resetOdometryConsumer;
    private Timer timer = new Timer();

    /**
     * Base robot config object required by PathPlanner to generate trajectories;
     * only used for visualization.
     */
    private RobotConfig pathPlannerRobotConfig = new RobotConfig(50, 1,
            new ModuleConfig(0.05, 4.5, 1.0, DCMotor.getKrakenX60(1), 60, 1));

    private AutoManager() {
    }

    /**
     * Returns the AutoManager instance.
     * 
     * @return the instance
     */
    public static AutoManager getInstance() {
        if (instance == null) {
            instance = new AutoManager();
        }
        return instance;
    }

    /**
     * Initializes logging for the AutoManager.
     */
    public void init() {
        chooser.setDefaultOption("None", new AutoRoutine("None", Commands.print("No path selected.")));
        LoggingManager.getInstance().addGroup(
                new LogGroup("autonomous",
                        new DoubleLogItem("autoTimer", timer::get, LogType.NT),
                        new SendableLogItem("field", field),
                        new SendableLogItem("chooser", chooser)));
    }

    /**
     * Add a routine to the manager.
     * 
     * @param routine the routine to add
     */
    public void addRoutine(AutoRoutine routine) {
        routines.put(routine.getName(), routine);
        chooser.addOption(routine.getName(), routine);
    }

    /**
     * Get the selected autonomous routine.
     * 
     * @return the selected autonomous routine
     */
    public AutoRoutine getSelectedRoutine() {
        return chooser.getSelected();
    }

    /**
     * Get the field object (used for the drivetrain to post odometry values).
     * 
     * @return the field object
     */
    public Field2d getField() {
        return field;
    }

    /**
     * Set the function associated with resetting odometry. This function must be
     * set if you are running an autonomous routine with paths or have any
     * dependencies on localization. This function should take in a {@code Pose2d}
     * and set the robot's pose to that value.
     * 
     * @param consumer the {@code Pose2d} consumer
     */
    public void setResetOdometryConsumer(Consumer<Pose2d> consumer) {
        this.resetOdometryConsumer = consumer;
    }

    /**
     * Sets the RobotConfig object to use when generating
     * 
     * @param consumer the {@code Pose2d} consumer
     */
    public void setRobotConfig(RobotConfig robotConfig) {
        this.pathPlannerRobotConfig = robotConfig;
    }

    /**
     * Updates the NetworkTables field with the new selected auto path. This
     * should be put in {@code disabledPeriodic()}.
     */
    public void periodicUpdate() {
        AutoRoutine selectedRoutine = getSelectedRoutine();
        if (getSelectedRoutine() != lastRoutine) {
            if (resetOdometryConsumer != null) {
                resetOdometryConsumer.accept(selectedRoutine.getInitialPose());
            }
            if (selectedRoutine.getPathPlannerPaths().size() > 0) {
                field.getObject("startingPose").setPose(selectedRoutine.getInitialPose());
                displayPaths(selectedRoutine.getPathPlannerPaths());
            }
            lastRoutine = selectedRoutine;
        }
    }

    /**
     * Display the selected routine's trajectories on the field object.
     */
    private void displayPaths(List<PathPlannerPath> paths) {
        ArrayList<Pose2d> poses = new ArrayList<Pose2d>();
        for (PathPlannerPath path : paths) {
            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                path = path.flipPath();
            }
            PathPlannerTrajectory trajectory = path.generateTrajectory(new ChassisSpeeds(),
                    path.getStartingDifferentialPose().getRotation(),
                    pathPlannerRobotConfig);

            poses.addAll(trajectory.getStates().stream()
                    .map((state) -> state.pose)
                    .collect(Collectors.toList()));
        }

        field.getObject("trajectory").setPoses(poses);
    }

    /**
     * Schedules the selected routine. This should be run in
     * {@code autonomousInit()}.
     */
    public void runSelectedRoutine() {
        if (this.resetOdometryConsumer == null) {
            DriverStation.reportError("[houndauto] No odometry reset consumer set.", false);
        }

        if (this.getSelectedRoutine() == null) {
            DriverStation.reportError("[houndauto] An auto routine must be chosen.", false);
        } else {
            // the line after this makes the autoroutine command a composition, and
            // commands that are in a composition cannot be recomposed, which is what this
            // would do if auto is run multiple times. this fixes it by removing the
            // composition from the scheduler.
            CommandScheduler.getInstance().removeComposedCommand(baseCommand);

            baseCommand = getSelectedRoutine().getCommand();

            timer.reset();
            timer.start();
            currentCommand = baseCommand
                    .beforeStarting(() -> resetOdometryConsumer.accept(getSelectedRoutine().getInitialPose()))
                    .andThen(Commands.runOnce(timer::stop))
                    .withName("auto");
            currentCommand.schedule();
        }
    }

    /**
     * Ends the selected routine. This should be run in {@code teleopInit()}.
     */
    public void endRoutine() {
        timer.stop();
        if (currentCommand != null) {
            currentCommand.cancel();
        }
    }
}
