package com.techhounds.houndutil.houndlib.robots;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.techhounds.houndutil.houndauto.AutoManager;
import com.techhounds.houndutil.houndlib.TriConsumer;
import com.techhounds.houndutil.houndlog.FaultLogger;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.SignalManager;

/**
 * A {@link TimedRobot} extension class that automatically sets up all of
 * HoundUtil for a typical robot project.
 * 
 * This class:
 * <ul>
 * <li>Initializes the {@link AutoManager}</li>
 * <li>Initializes the {@link LoggingManager}</li>
 * <li>Sets up the {@link FaultLogger} to run on 0.100s period</li>
 * <li>Disables LiveWindow</li>
 * <li>Finalizes all {@code BaseStatusSignal}s registered with the
 * {@link SignalManager}</li>
 * <li>Runs the {@link CommandScheduler} periodically</li>
 * <li>Runs the {@link LoggingManager} periodically</li>
 * <li>Refreshes all signals registered with the {@link SignalManager}
 * periodically</li>
 * <li>Updates the {@link AutoManager} NetworkTables GUI while disabled</li>
 * <li>Executes the selected autonomous routine at the start of the autonomous
 * period</li>
 * </ul>
 * 
 * To use, delete the {@code Robot.java} file and replace
 * {@code Main.java} with the following:
 * 
 * <pre>
 * public final class Main {
 *     private Main() {
 *     }
 * 
 *     public static void main(String... args) {
 *         RobotBase.startRobot(() -> new HoundRobot(() -> new RobotContainer()));
 *     }
 * }
 * </pre>
 * 
 * If any modifications are needed, create use anonymous inner classes.
 */
public class HoundRobot extends TimedRobot {
    /**
     * Default constructor. Use if not using a RobotContainer, or initializing a
     * RobotContainer elsewhere.
     */
    public HoundRobot() {
    }

    public HoundRobot(Supplier<Object> robotContainerCtor) {
        if (robotContainerCtor != null)
            robotContainerCtor.get();
    }

    public HoundRobot(Consumer<TriConsumer<Runnable, Double, Double>> robotContainerCtor) {
        if (robotContainerCtor != null)
            robotContainerCtor.accept((callback, periodSeconds, offsetSeconds) -> {
                addPeriodic(callback, periodSeconds, offsetSeconds);
            });
    }

    /**
     * Initializes HoundUtil.
     */
    @Override
    public void robotInit() {
        AutoManager.getInstance().init();
        LoggingManager.getInstance().init();
        addPeriodic(FaultLogger::update, 0.100, 0.010);
        SignalManager.finalizeAll();

        // LiveWindow is essentially deprecated, and HoundLog is a much better
        // replacement. LiveWindow is still active during test mode by default, but it
        // consumes an inordinate amount of bandwidth, so we disable it.
        LiveWindow.disableAllTelemetry();
    }

    /**
     * Runs commands and logging periodically.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        LoggingManager.getInstance().run();
        SignalManager.refresh();
    }

    @Override
    public void disabledInit() {
    }

    /**
     * Updates the AutoManager NetworkTables GUI periodically, only when disabled.
     */
    @Override
    public void disabledPeriodic() {
        AutoManager.getInstance().periodicUpdate();
    }

    /**
     * Schedules the selected autonomous routine when enabled.
     */
    @Override
    public void autonomousInit() {
        AutoManager.getInstance().runSelectedRoutine();
    }

    @Override
    public void autonomousPeriodic() {
    }

    /**
     * Cancels the selected autonomous routine when disabled.
     */
    @Override
    public void autonomousExit() {
        AutoManager.getInstance().endRoutine();
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
    }

    /**
     * Cancels all currently running commands (mostly for safety).
     */
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        LiveWindow.setEnabled(false);
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
