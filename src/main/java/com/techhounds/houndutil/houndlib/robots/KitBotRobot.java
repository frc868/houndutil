package com.techhounds.houndutil.houndlib.robots;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LogProfileBuilder;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.loggers.DeviceLogger;
import com.techhounds.houndutil.houndlog.loggers.SendableLogger;
import com.techhounds.houndutil.houndlog.logitems.TunableDouble;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class KitBotRobot extends HoundRobot {
    protected CANSparkMax leftMotor;
    protected CANSparkMax rightMotor;
    private DifferentialDrive drive;
    private XboxController controller = new XboxController(0);
    private TunableDouble speedLimit = new TunableDouble("Main", "Speed Limit", 0.5);

    public KitBotRobot(int leftMotorId, int rightMotorId) {
        super();

        leftMotor = new CANSparkMax(leftMotorId, MotorType.kBrushless);
        rightMotor = new CANSparkMax(rightMotorId, MotorType.kBrushless);
        rightMotor.setInverted(true);
        drive = new DifferentialDrive(leftMotor, rightMotor);
        LoggingManager.getInstance().addGroup("Drivetrain", new LogGroup(
                new DeviceLogger("Left Motor", LogProfileBuilder.buildCANSparkMaxLogItems(leftMotor)),
                new DeviceLogger("Right Motor", LogProfileBuilder.buildCANSparkMaxLogItems(rightMotor)),
                new SendableLogger("Drive", drive)));
    }

    @Override
    public void teleopPeriodic() {
        super.teleopPeriodic();
        drive.tankDrive(speedLimit.get() * -controller.getLeftY(), speedLimit.get() * -controller.getRightY());
    }
}
