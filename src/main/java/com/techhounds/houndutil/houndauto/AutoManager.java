package com.techhounds.houndutil.houndauto;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Consumer;
import java.util.stream.Collectors;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.enums.LogType;
import com.techhounds.houndutil.houndlog.loggers.SendableLogger;
import com.techhounds.houndutil.houndlog.logitems.DoubleLogItem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AutoManager {
    private static AutoManager instance;
    private HashMap<String, AutoRoutine> routines = new HashMap<String, AutoRoutine>();
    private AutoRoutine lastRoutine;
    private SendableChooser<AutoRoutine> chooser = new SendableChooser<AutoRoutine>();
    private Field2d field = new Field2d();
    private Command baseCommand;
    private Command currentCommand;
    private Consumer<Pose2d> resetOdometryConsumer;
    private HashMap<String, Command> eventMap = new HashMap<String, Command>();

    private Timer timer = new Timer();

    /**
     * Initialize the AutoManager.
     * Set up the Shuffleboard tab and display a trajectory if one is already set in
     * the chooser.
     */
    private AutoManager() {
        // lastRoutine = chooser.getSelected();
        chooser.setDefaultOption("None", new AutoRoutine("None", Commands.print("Waiting...")));
        setupShuffleboardTab();
    }

    /**
     * Returns a singleton of AutonManager.
     * 
     * @return a singleton AutonManager.
     */
    public static AutoManager getInstance() {
        if (instance == null) {
            instance = new AutoManager();
        }
        return instance;
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
     * Set the function associated with resetting odometry. This function should
     * take in a {@code Pose2d}, reset the odometry, and reset the encoders.
     * 
     * @param consumer the {@code Pose2d} consumer
     */
    public void setResetOdometryConsumer(Consumer<Pose2d> consumer) {
        this.resetOdometryConsumer = consumer;
    }

    /**
     * Create a Shuffleboard tab with the field and chooser objects.
     */
    public void setupShuffleboardTab() {
        LoggingManager.getInstance().addGroup("autonomous",
                new LogGroup(
                        new DoubleLogItem("autoTimer", timer::get, LogType.NT),
                        new SendableLogger("field", field),
                        new SendableLogger("chooser", chooser)));
    }

    /**
     * Updates the Shuffleboard visualization with the new selected auton path. This
     * should be put in {@code disabledPeriodic()}.
     */
    public void periodicUpdate() {
        AutoRoutine selectedRoutine = getSelectedRoutine();
        if (getSelectedRoutine() != lastRoutine) {
            if (resetOdometryConsumer != null) {
                resetOdometryConsumer.accept(selectedRoutine.getInitialPose());
            }
            if (selectedRoutine.getPathPlannerPaths() != null)
                displayPaths(selectedRoutine.getPathPlannerPaths());
            lastRoutine = selectedRoutine;
        }
    }

    /**
     * Display the selected routine's trajectories on the field object.
     */
    public void displayPaths(List<PathPlannerPath> paths) {
        ArrayList<Pose2d> poses = new ArrayList<Pose2d>();
        for (PathPlannerPath path : paths) {
            PathPlannerTrajectory trajectory = path.getTrajectory(new ChassisSpeeds(),
                    path.getStartingDifferentialPose().getRotation());

            poses.addAll(trajectory.getStates().stream()
                    .map((state) -> new Pose2d(state.positionMeters, state.targetHolonomicRotation))
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
            throw new NullPointerException(
                    "Reset Odometry Consumer must not be null, set it in the drivetrain constructor.");
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
                    .withName("auto");
            currentCommand.schedule();
            new Trigger(currentCommand::isFinished).onTrue(Commands.runOnce(timer::stop).ignoringDisable(true));
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

    public void addEvent(String key, Command command) {
        eventMap.put(key, command);
    }

    public HashMap<String, Command> getEventMap() {
        if (eventMap.size() == 0) {
            throw new NullPointerException("You must add events to the event map!");
        }
        return eventMap;
    }
}
