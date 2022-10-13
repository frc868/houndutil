package com.techhounds.houndutil.houndlib.auto;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoManager {
    private static AutoManager instance;
    private HashMap<String, AutoRoutine> routines = new HashMap<String, AutoRoutine>();
    private AutoRoutine lastRoutine;
    private SendableChooser<AutoRoutine> chooser = new SendableChooser<AutoRoutine>();
    private Field2d field = new Field2d();
    private Command scheduledCommand;
    private Consumer<Pose2d> resetOdometryConsumer;

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
        ShuffleboardTab tab = Shuffleboard.getTab("Autonomous");
        tab.add("Field", field);
        tab.add("Auto Chooser", chooser);
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
                ArrayList<AutoPath> autoPaths = selectedRoutine.getTrajectories();
                // set the robot odometry to the initial pose of the first trajectory in the
                // routine
                resetOdometryConsumer.accept(autoPaths.get(0).getTrajectory().getInitialPose());
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
            getSelectedRoutine().getTrajectories().forEach((autoPath) -> {
                field.getObject(autoPath.getName()).setTrajectory(autoPath.getTrajectory());
            });
        }
    }

    /**
     * Remove the last routine's trajectories from the field object. Currently there
     * is a problem with removing them that will be fixed in NT4.
     */
    public void removeLastRoutine(AutoRoutine last) {
        System.out.println("removing last " + last.getName());
        last.getTrajectories()
                .forEach((autoPath) -> {
                    field.getObject(autoPath.getName()).setTrajectory(new Trajectory());
                });
    }

    /**
     * Schedules the selected routine. This should be run in
     * {@code autonomousInit()}.
     */
    public void runSelectedRoutine() {
        scheduledCommand = getSelectedRoutine().getCommand().beforeStarting(() -> resetOdometryConsumer
                .accept(getSelectedRoutine().getTrajectories().get(0).getTrajectory().getInitialPose()));
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
}
