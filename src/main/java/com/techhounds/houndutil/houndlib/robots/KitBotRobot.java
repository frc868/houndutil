package com.techhounds.houndutil.houndlib.robots;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class KitBotRobot extends TimedRobot {
    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    private DifferentialDrive drive;
    private XboxController controller = new XboxController(0);

    public KitBotRobot(int leftMotorId, int rightMotorId) {
        leftMotor = new CANSparkMax(leftMotorId, MotorType.kBrushless);
        rightMotor = new CANSparkMax(rightMotorId, MotorType.kBrushless);
        rightMotor.setInverted(true);
        drive = new DifferentialDrive(leftMotor, rightMotor);
    }

    @Override
    public void teleopPeriodic() {
        drive.tankDrive(-controller.getLeftY(), -controller.getRightY());
    }

}
