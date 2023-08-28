package com.techhounds.houndutil.houndlib.robots;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.function.Supplier;

import com.techhounds.houndutil.houndauto.AutoManager;
import com.techhounds.houndutil.houndlog.LoggingManager;

public class HoundRobot extends TimedRobot {
    public HoundRobot() {
    }

    public HoundRobot(Supplier<Object> robotContainerSupplier) {
        if (robotContainerSupplier != null)
            robotContainerSupplier.get(); // calls `new RobotContainer()` basically.
    }

    @Override
    public void robotInit() {
        AutoManager.getInstance().setupShuffleboardTab();

        // sets the LoggingManager to run every 100ms and on an offset of 10ms from the
        // main thread
        LoggingManager.getInstance().init();
        addPeriodic(LoggingManager.getInstance()::run, 0.1, 0.010);
        if (RobotBase.isReal())
            addPeriodic(AutoManager.getInstance()::updatePoseEstimator, 0.1, 0.010);

        LiveWindow.disableAllTelemetry();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
        AutoManager.getInstance().periodicUpdate(false);
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
