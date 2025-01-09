package com.techhounds.houndutil.houndlib.robots;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.techhounds.houndutil.houndlog.LogProfiles;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.loggers.LogGroup;
import com.techhounds.houndutil.houndlog.loggers.SendableLogItem;
import com.techhounds.houndutil.houndlog.loggers.TunableDouble;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * A {@link TimedRobot} extension class that creates all of the resources
 * necessary for a low-code time-based tank-drive KitBot using SPARK MAXes.
 * Intended as an example project.
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
 *         RobotBase.startRobot(() -> new KitBotRobot(1, 2));
 *     }
 * }
 * </pre>
 * 
 * The contents of this class can also be copied directly into a
 * {@code Robot.java} for further functionality, or you may use extend this
 * class to modify functionality.
 * 
 * @apiNote If this is a 4 wheel robot, set the SPARK MAXes into a follower
 *          configuration via the REV Hardware Client.
 */
public class KitBotRobot extends HoundRobot {
    /**
     * The motor controller for the left side of the drivetrain.
     */
    protected SparkMax leftMotor;
    /** The motor controller for the right side of the drivetrain. */
    protected SparkMax rightMotor;
    /**
     * The helper object to apply inputs to the motor controllers based on the
     * joysticks.
     */
    private DifferentialDrive drive;
    /** The Xbox controller. */
    private XboxController controller = new XboxController(0);
    /** The NetworkTables-based speed limit for the robot. */
    private TunableDouble speedLimit;

    /**
     * Creates a KitBotRobot object and initializes the motor controllers.
     * 
     * @param leftMotorId       the ID of the motor controller on the left side of
     *                          the drivetrain, or the ID of the main motor
     *                          controller in the follow group.
     * @param rightMotorId      the ID of the motor controller on the right side of
     *                          the drivetrain, or the ID of the main motor
     *                          controller in the follow group.
     * @param initialSpeedLimit the initial speed limit (0, 1] to apply to the
     *                          robot. 0.5 indicates a maximum of 50% speed.
     */
    public KitBotRobot(int leftMotorId, int rightMotorId, double initialSpeedLimit) {
        super();

        leftMotor = new SparkMax(leftMotorId, MotorType.kBrushless);
        rightMotor = new SparkMax(rightMotorId, MotorType.kBrushless);
        // TODO: deprecated
        rightMotor.setInverted(true);
        speedLimit = new TunableDouble("main/speedLimit", initialSpeedLimit);
        drive = new DifferentialDrive(leftMotor, rightMotor);
        LoggingManager.getInstance().addGroup(new LogGroup("drivetrain",
                new LogGroup("leftMotor", LogProfiles.logSparkMax(leftMotor)),
                new LogGroup("rightMotor", LogProfiles.logSparkMax(rightMotor)),
                new SendableLogItem("drive", drive)));
    }

    @Override
    public void teleopPeriodic() {
        super.teleopPeriodic();
        drive.tankDrive(speedLimit.get() * -controller.getLeftY(), speedLimit.get() * -controller.getRightY());
    }
}
