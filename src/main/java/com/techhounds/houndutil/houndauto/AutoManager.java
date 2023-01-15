package com.techhounds.houndutil.houndauto;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Consumer;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.loggers.SendableLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class AutoManager {
    private static AutoManager instance;
    private HashMap<String, AutoRoutine> routines = new HashMap<String, AutoRoutine>();
    private AutoRoutine lastRoutine;
    private SendableChooser<AutoRoutine> chooser = new SendableChooser<AutoRoutine>();
    private Field2d field = new Field2d();
    private Command scheduledCommand;
    private Consumer<Pose2d> resetOdometryConsumer;
    private HashMap<String, Command> eventMap = new HashMap<String, Command>();

    /**
     * Initialize the AutoManager.
     * Set up the Shuffleboard tab and display a trajectory if one is already set in
     * the chooser.
     */
    private AutoManager() {
        lastRoutine = chooser.getSelected();
        setupShuffleboardTab();
        displaySelectedRoutine();
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
        LoggingManager.getInstance().addGroup("Autonomous",
                new LogGroup(
                        new SendableLogger("Field", field),
                        new SendableLogger("Chooser", chooser)));
    }

    /**
     * Updates the Shuffleboard visualization with the new selected auton path. This
     * should be put in {@code disabledPeriodic()}.
     */
    public void updateShuffleboard() {
        AutoRoutine selectedRoutine = getSelectedRoutine();
        if (getSelectedRoutine() != lastRoutine) {
            if (lastRoutine != null) {
                removeLastRoutine(lastRoutine);
            }
            if (resetOdometryConsumer != null) {
                AutoPath autoPath = selectedRoutine.getAutoPath();
                // set the robot odometry to the initial pose of the first trajectory in the
                // routine
                resetOdometryConsumer.accept(autoPath.getTrajectories().get(0).getInitialPose());
            }
            displaySelectedRoutine();
            lastRoutine = selectedRoutine;
        }
    }

    /**
     * Display the selected routine's trajectories on the field object.
     */
    public void displaySelectedRoutine() {
        if (getSelectedRoutine() != null) {
            ArrayList<PathPlannerTrajectory> trajectories = getSelectedRoutine().getAutoPath().getTrajectories();
            for (int i = 0; i < trajectories.size(); i++) {
                field.getObject(getSelectedRoutine().getAutoPath().getName() + "_" + i)
                        .setTrajectory(trajectories.get(i));
            }
        }
    }

    /**
     * Remove the last routine's trajectories from the field object. Currently there
     * is a problem with removing them that will be fixed in NT4.
     */
    public void removeLastRoutine(AutoRoutine last) {
        ArrayList<PathPlannerTrajectory> trajectories = last.getAutoPath().getTrajectories();
        for (int i = 0; i < trajectories.size(); i++) {
            field.getObject(last.getAutoPath().getName() + "_" + i)
                    .setTrajectory(new Trajectory());
        }
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
            throw new NullPointerException("An auto routine must be chosen.");
        }

        // the next line after this makes the autoroutine command a composition, and
        // commands that are in a composition cannot be recomposed, which is what this
        // would do if auto is run multiple times. this fixes it by removing the
        // composition from the scheduler.
        CommandScheduler.getInstance().removeComposedCommand(getSelectedRoutine().getCommand());
        scheduledCommand = getSelectedRoutine().getCommand().beforeStarting(() -> resetOdometryConsumer
                .accept(getSelectedRoutine().getAutoPath().getTrajectories().get(0).getInitialPose()));
        scheduledCommand.schedule();
    }

    /**
     * Ends the selected routine. This should be run in {@code teleopInit()}.
     */
    public void endRoutine() {
        if (scheduledCommand != null) {
            scheduledCommand.cancel();
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
