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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ProxyCommand;

public class AutoManager {
    private static AutoManager instance;
    private HashMap<String, AutoRoutine> routines = new HashMap<String, AutoRoutine>();
    private AutoRoutine lastRoutine;
    private SendableChooser<AutoRoutine> chooser = new SendableChooser<AutoRoutine>();
    private Field2d field = new Field2d();
    private Command scheduledCommand;
    private Consumer<Pose2d> resetOdometryConsumer;
    private HashMap<String, Command> eventMap = new HashMap<String, Command>();

    private Runnable updatePoseEstimatorCallback = null;

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
    public void updateShuffleboard(boolean ignoreLastTrajCheck) {
        AutoRoutine selectedRoutine = getSelectedRoutine();
        if (getSelectedRoutine() != lastRoutine || ignoreLastTrajCheck) {
            if (resetOdometryConsumer != null) {
                try {
                    resetOdometryConsumer.accept(selectedRoutine.getCommand().get().getInitialPosition());
                } catch (Exception e) {
                    DriverStation.reportError("Error resetting odometry: " + e.getMessage(), true);
                }
            }
            displaySelectedRoutine();
            lastRoutine = selectedRoutine;
        }
    }

    /**
     * Display the selected routine's trajectories on the field object.
     */
    public void displaySelectedRoutine() {
        if (getSelectedRoutine() != null && getSelectedRoutine().getCommand().get().getAutoPath().isPresent()) {
            System.out.println(getSelectedRoutine().getName());
            System.out.println(getSelectedRoutine().getCommand().get().getName());

            ArrayList<PathPlannerTrajectory> trajectories = getSelectedRoutine().getCommand().get().getAutoPath()
                    .orElseThrow().getTrajectories();
            Trajectory fullTrajectory = trajectories.get(0);
            for (int i = 1; i < trajectories.size(); i++) {
                fullTrajectory = fullTrajectory.concatenate(trajectories.get(i));
            }
            field.getObject("Autonomous Routine")
                    .setTrajectory(fullTrajectory);
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
            DriverStation.reportError("[houndauto] An auto routine must be chosen.", false);
        } else {

            // the line after this makes the autoroutine command a composition, and
            // commands that are in a composition cannot be recomposed, which is what this
            // would do if auto is run multiple times. this fixes it by removing the
            // composition from the scheduler.
            CommandScheduler.getInstance().removeComposedCommand(scheduledCommand);

            ProxyCommand proxiedCommand = new ProxyCommand(() -> getSelectedRoutine().getCommand().get().beforeStarting(
                    () -> resetOdometryConsumer
                            .accept(getSelectedRoutine().getCommand().get().getInitialPosition())));
            proxiedCommand.schedule();
        }
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

    public void setPoseEstimatorCallback(Runnable callback) {
        this.updatePoseEstimatorCallback = callback;
    }

    public void updatePoseEstimator() {
        if (updatePoseEstimatorCallback != null) {
            this.updatePoseEstimatorCallback.run();
        }
    }
}
