package com.techhounds.houndutil.houndlib.robots;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.techhounds.houndutil.houndlog.loggers.TunableNumber;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class KitBotRobot extends HoundRobot {
    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    private DifferentialDrive drive;
    private XboxController controller = new XboxController(0);
    private TunableNumber speedLimit = new TunableNumber("Main", "Speed Limit", 0.5);

    public KitBotRobot(int leftMotorId, int rightMotorId) {
        leftMotor = new CANSparkMax(leftMotorId, MotorType.kBrushless);
        rightMotor = new CANSparkMax(rightMotorId, MotorType.kBrushless);
        rightMotor.setInverted(true);
        drive = new DifferentialDrive(leftMotor, rightMotor);
    }

    @Override
    public void teleopPeriodic() {
        super.teleopPeriodic();
        drive.tankDrive(speedLimit.get() * -controller.getLeftY(), speedLimit.get() * -controller.getRightY());
    }
}
