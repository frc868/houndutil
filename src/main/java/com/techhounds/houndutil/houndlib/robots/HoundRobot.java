package com.techhounds.houndutil.houndlib.robots;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.techhounds.houndutil.houndauto.AutoManager;
import com.techhounds.houndutil.houndlib.TriConsumer;
import com.techhounds.houndutil.houndlog.LoggingManager;

public class HoundRobot extends TimedRobot {
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

    @Override
    public void robotInit() {
        AutoManager.getInstance().setupShuffleboardTab();

        // sets the LoggingManager to run every 100ms and on an offset of 10ms from the
        // main thread
        LoggingManager.getInstance().init();
        LiveWindow.disableAllTelemetry();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        LoggingManager.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
        AutoManager.getInstance().periodicUpdate();
    }

    @Override
    public void autonomousInit() {
        AutoManager.getInstance().runSelectedRoutine();
    }

    @Override
    public void autonomousPeriodic() {
    }

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
