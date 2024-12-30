package com.techhounds.houndutil.houndlog;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import com.techhounds.houndutil.houndlog.annotations.LogProfile;
import com.techhounds.houndutil.houndlog.loggers.BooleanLogItem;
import com.techhounds.houndutil.houndlog.loggers.DoubleArrayLogItem;
import com.techhounds.houndutil.houndlog.loggers.DoubleLogItem;
import com.techhounds.houndutil.houndlog.loggers.FloatLogItem;
import com.techhounds.houndutil.houndlog.loggers.IntegerArrayLogItem;
import com.techhounds.houndutil.houndlog.loggers.IntegerLogItem;
import com.techhounds.houndutil.houndlog.loggers.LogItem;
import com.techhounds.houndutil.houndlog.loggers.TunableDouble;

/**
 * Creates an array of {@link LogItem}s given . Logs useful information about
 * each device.
 * Reduces verbosity by a TON.
 * 
 * @author dr
 */
public class LogProfiles {

    public static String revFaultsToString(short value) {
        StringBuilder result = new StringBuilder(16);

        for (int i = 15; i >= 0; i--) {
            // Check each bit using a bitmask
            boolean isBitSet = (value & (1 << i)) != 0;
            if (isBitSet) {
                result.append("*"); // Append '*' for 1
            } else {
                result.append("."); // Append '.' for 0
            }
        }

        return result.toString();
    }

    public static String ctreFaultsToString(int value) {
        StringBuilder result = new StringBuilder(24);

        for (int i = 23; i >= 0; i--) {
            // Check each bit using a bitmask
            boolean isBitSet = (value & (1 << i)) != 0;
            if (isBitSet) {
                result.append("*"); // Append '*' for 1
            } else {
                result.append("."); // Append '.' for 0
            }
        }

        return result.toString();
    }

    /**
     * Builds CANSparkMax log items.
     * 
     * @param obj the CANSparkMax object to use
     * @return the array of LogItems
     */
    @LogProfile(TalonFX.class)
    public static LogItem<?>[] logTalonFX(TalonFX obj) {
        StatusSignal<Double> position = obj.getPosition();
        StatusSignal<Double> velocity = obj.getVelocity();
        StatusSignal<Double> acceleration = obj.getAcceleration();
        StatusSignal<Double> temp = obj.getDeviceTemp();
        StatusSignal<Double> outputVoltage = obj.getMotorVoltage();
        StatusSignal<Double> outputCurrent = obj.getTorqueCurrent();

        SignalManager.register(position, velocity, acceleration, temp, outputVoltage, outputCurrent);
        FaultLogger.register(obj);
        return new LogItem<?>[] {
                new DoubleLogItem("position", () -> position.getValue(), LogType.NT),
                new DoubleLogItem("velocity", () -> velocity.getValue(), LogType.NT),
                new DoubleLogItem("acceleration", () -> acceleration.getValue(), LogType.NT),
                new DoubleLogItem("temperature", () -> temp.getValue(), LogType.NT),
                new DoubleLogItem("outputVoltage", () -> outputVoltage.getValue(), LogType.NT),
                new DoubleLogItem("outputCurrent", () -> outputCurrent.getValue(), LogType.NT),
                new DoubleLogItem("outputCurrent", () -> outputCurrent.getValue(), LogType.NT),
        };
    }

    @LogProfile(CANSparkFlex.class)
    public static LogItem<?>[] logCANSparkFlex(CANSparkFlex obj) {
        FaultLogger.register(obj);
        return logCANSparkBase(obj);
    }

    @LogProfile(CANSparkMax.class)
    public static LogItem<?>[] logCANSparkMax(CANSparkMax obj) {
        FaultLogger.register(obj);
        return logCANSparkBase(obj);
    }

    private static LogItem<?>[] logCANSparkBase(CANSparkBase obj) {
        return new LogItem<?>[] {
                new DoubleLogItem("encoderPosition", obj.getEncoder()::getPosition, LogType.NT),
                new DoubleLogItem("encoderVelocity", obj.getEncoder()::getVelocity, LogType.NT),
                new DoubleLogItem("outputVoltage",
                        () -> RobotBase.isReal() ? obj.getAppliedOutput() * obj.getBusVoltage()
                                : obj.getAppliedOutput(),
                        LogType.NT),
                new DoubleLogItem("motorTemperature", obj::getMotorTemperature, LogType.NT),
                new DoubleLogItem("outputCurrent", obj::getOutputCurrent, LogType.NT)
        };
    }

    /**
     * Builds CANSparkMax log items.
     * 
     * @param obj the CANSparkMax object to use
     * @return the array of LogItems
     */
    @LogProfile(CANcoder.class)
    public static LogItem<?>[] logCANcoder(CANcoder obj) {
        StatusSignal<Double> absolutePosition = obj.getAbsolutePosition();
        StatusSignal<Double> position = obj.getPosition();
        StatusSignal<Double> velocity = obj.getVelocity();

        SignalManager.register(absolutePosition, position, velocity);
        FaultLogger.register(obj);
        return new LogItem<?>[] {
                new DoubleLogItem("absolutePosition", () -> absolutePosition.getValue(), LogType.NT),
                new DoubleLogItem("position", () -> position.getValue(), LogType.NT),
                new DoubleLogItem("velocity", () -> velocity.getValue(), LogType.NT),
        };
    }

    /**
     * Builds CANSparkMax log items.
     * 
     * @param obj the CANSparkMax object to use
     * @return the array of LogItems
     */
    @LogProfile(SparkAbsoluteEncoder.class)
    public static LogItem<?>[] logSparkAbsoluteEncoder(SparkAbsoluteEncoder obj) {
        return new LogItem<?>[] {
                new DoubleLogItem("position", obj::getPosition, LogType.NT),
                new DoubleLogItem("velocity", obj::getVelocity, LogType.NT),
                new DoubleLogItem("zeroOffset", obj::getZeroOffset, LogType.NT),
        };
    }

    /**
     * Builds NavX log items.
     * 
     * @param obj the navx to use
     * @return the array of LogItems
     */
    @LogProfile(AHRS.class)
    public static LogItem<?>[] logNavX(AHRS obj) {
        return new LogItem<?>[] {
                new FloatLogItem("pitch", obj::getPitch, LogType.NT),
                new FloatLogItem("roll", obj::getRoll, LogType.NT),
                new FloatLogItem("yaw", obj::getYaw, LogType.NT),
                new DoubleLogItem("yawRotationRate", obj::getRate, LogType.NT),
                new FloatLogItem("xAcceleration", obj::getWorldLinearAccelX, LogType.DATALOG),
                new FloatLogItem("yAcceleration", obj::getWorldLinearAccelY, LogType.DATALOG),
                new FloatLogItem("zAcceleration", obj::getWorldLinearAccelZ, LogType.DATALOG),
                new FloatLogItem("compassHeading", obj::getCompassHeading, LogType.NT),
                new BooleanLogItem("isCalibrating", obj::isCalibrating, LogType.NT),
                new BooleanLogItem("isMagnetometerCalibrated", obj::isMagnetometerCalibrated,
                        LogType.NT),
                new BooleanLogItem("isConnected", obj::isConnected, LogType.DATALOG),
                new BooleanLogItem("isMoving", obj::isMoving, LogType.NT),
                new BooleanLogItem("isRotating", obj::isRotating, LogType.NT),
                new BooleanLogItem("isMagneticDisturbance", obj::isMagneticDisturbance,
                        LogType.DATALOG),
                new FloatLogItem("temperature", obj::getTempC, LogType.NT),
                new DoubleLogItem("updateCount", obj::getUpdateCount, LogType.DATALOG),
        };
    }

    /**
     * Builds Pigeon 2 log items.
     * 
     * @param obj the navx to use
     * @return the array of LogItems
     */
    @LogProfile(Pigeon2.class)
    public static LogItem<?>[] logPigeon2(Pigeon2 obj) {
        StatusSignal<Double> pitch = obj.getPitch();
        StatusSignal<Double> roll = obj.getRoll();
        StatusSignal<Double> yaw = obj.getYaw();

        SignalManager.register(pitch, roll, yaw);
        FaultLogger.register(obj);
        return new LogItem<?>[] {
                new DoubleLogItem("pitch", () -> pitch.getValue(), LogType.NT),
                new DoubleLogItem("roll", () -> roll.getValue(), LogType.NT),
                new DoubleLogItem("yaw", () -> yaw.getValue(), LogType.NT),
                new DoubleLogItem("yawRad", () -> Units.degreesToRadians(yaw.getValue()), LogType.NT),
        };
    }

    /**
     * Builds DoubleSolenoid log items.
     * 
     * @param obj the {@link DoubleSolenoid} to use
     * @return the array of LogItems
     */
    public static LogItem<?>[] logDoubleSolenoid(DoubleSolenoid obj) {
        return new LogItem<?>[] {
                new BooleanLogItem("position", () -> obj.get() == DoubleSolenoid.Value.kForward,
                        LogType.NT),
                new BooleanLogItem("isFwdSolenoidDisabled", obj::isFwdSolenoidDisabled,
                        LogType.DATALOG),
                new BooleanLogItem("isRevSolenoidDisabled", obj::isRevSolenoidDisabled,
                        LogType.DATALOG)
        };
    }

    /**
     * Builds PDH log items.
     * 
     * @param obj the PDH instance to use
     * @return the array of LogItems
     */
    @LogProfile(PowerDistribution.class)
    public static LogItem<?>[] logPDH(PowerDistribution obj) {
        FaultLogger.register(obj);
        return new LogItem<?>[] {
                new DoubleLogItem("voltage", obj::getVoltage, LogType.NT),
                new DoubleLogItem("temperature", obj::getTemperature, LogType.NT),
                new DoubleLogItem("totalCurrentAmps", obj::getTotalCurrent, LogType.NT),
                new DoubleArrayLogItem("channelCurrents", () -> new double[] {
                        obj.getCurrent(0),
                        obj.getCurrent(1),
                        obj.getCurrent(2),
                        obj.getCurrent(3),
                        obj.getCurrent(4),
                        obj.getCurrent(5),
                        obj.getCurrent(6),
                        obj.getCurrent(7),
                        obj.getCurrent(8),
                        obj.getCurrent(9),
                        obj.getCurrent(10),
                        obj.getCurrent(11),
                        obj.getCurrent(12),
                        obj.getCurrent(13),
                        obj.getCurrent(14),
                        obj.getCurrent(15),
                        obj.getCurrent(16),
                        obj.getCurrent(17),
                        obj.getCurrent(18),
                        obj.getCurrent(19),
                        obj.getCurrent(20),
                        obj.getCurrent(21),
                        obj.getCurrent(22),
                        obj.getCurrent(23),
                }, LogType.DATALOG),
        };
    }

    /**
     * Builds REV Pneumatics Hub log items.
     * 
     * @param obj the PH instance to use
     * @return the array of LogItems
     */
    @LogProfile(PneumaticHub.class)
    public static LogItem<?>[] logPneumaticHubLog(PneumaticHub obj) {
        FaultLogger.register(obj);
        return new LogItem<?>[] {
                new DoubleLogItem("inputVoltage", obj::getInputVoltage, LogType.NT),
                new DoubleLogItem("regulatedVoltage", obj::get5VRegulatedVoltage, LogType.NT),

                new DoubleArrayLogItem("pressures", () -> new double[] {
                        obj.getPressure(0),
                        obj.getPressure(1),
                        obj.getPressure(2),
                        obj.getPressure(3),
                        obj.getPressure(4),
                        obj.getPressure(5),
                        obj.getPressure(6),
                        obj.getPressure(7),
                        obj.getPressure(8),
                        obj.getPressure(9),
                        obj.getPressure(10),
                        obj.getPressure(11),
                        obj.getPressure(12),
                        obj.getPressure(13),
                        obj.getPressure(14),
                        obj.getPressure(15),
                }, LogType.NT),
                new BooleanLogItem("pressureSwitch", obj::getPressureSwitch, LogType.DATALOG),
                new DoubleLogItem("compressorCurrent", obj::getCompressorCurrent, LogType.NT),
                new BooleanLogItem("isCompressorRunning", obj::getCompressor, LogType.NT),
                new DoubleLogItem("solenoidsTotalCurrent", obj::getSolenoidsTotalCurrent, LogType.DATALOG),
                new DoubleLogItem("solenoidsVoltage", obj::getSolenoidsVoltage, LogType.DATALOG),

        };
    }

    /**
     * Builds PIDController log items.
     * 
     * @param obj the {@link PIDController} to use
     * @return the array of LogItems
     */
    @LogProfile(PIDController.class)
    public static LogItem<?>[] logPIDController(PIDController obj) {
        return new LogItem<?>[] {
                new DoubleLogItem("setpoint", () -> obj.getSetpoint(),
                        LogType.NT),
                new BooleanLogItem("atSetpoint", () -> obj.atSetpoint(),
                        LogType.NT),
                new DoubleLogItem("positionError", () -> obj.getPositionError(), LogType.NT),
                new TunableDouble("tunables/kP", obj.getP(), (d) -> obj.setP(d)),
                new TunableDouble("tunables/kI", obj.getI(), (d) -> obj.setI(d)),
                new TunableDouble("tunables/kD", obj.getD(), (d) -> obj.setD(d)),
                new TunableDouble("tunables/positionTolerance", obj.getPositionTolerance(),
                        (d) -> obj.setTolerance(d)),
                new TunableDouble("tunables/velocityTolerance", obj.getVelocityTolerance(),
                        (d) -> obj.setTolerance(obj.getPositionTolerance(), d)),
        };
    }

    /**
     * Builds ProfiledPIDController log items.
     * 
     * @param obj the {@link PIDController} to use
     * @return the array of LogItems
     */
    @LogProfile(ProfiledPIDController.class)
    public static LogItem<?>[] logProfiledPIDController(ProfiledPIDController obj) {
        return new LogItem<?>[] {
                new DoubleLogItem("setpointPosition", () -> obj.getSetpoint().position,
                        LogType.NT),
                new DoubleLogItem("setpointVelocity", () -> obj.getSetpoint().velocity,
                        LogType.NT),
                new BooleanLogItem("atSetpoint", () -> obj.atSetpoint(),
                        LogType.NT),
                new DoubleLogItem("goalPosition", () -> obj.getGoal().position,
                        LogType.NT),
                new DoubleLogItem("goalVelocity", () -> obj.getGoal().velocity,
                        LogType.NT),
                new BooleanLogItem("atGoal", () -> obj.atGoal(),
                        LogType.NT),
                new DoubleLogItem("positionError", () -> obj.getPositionError(),
                        LogType.NT),
                new TunableDouble("tunables/kP", obj.getP(), (d) -> obj.setP(d)),
                new TunableDouble("tunables/kI", obj.getI(), (d) -> obj.setI(d)),
                new TunableDouble("tunables/kD", obj.getD(), (d) -> obj.setD(d)),
                new TunableDouble("tunables/positionTolerance", obj.getPositionTolerance(),
                        (d) -> obj.setTolerance(d)),
                new TunableDouble("tunables/velocityTolerance", obj.getVelocityTolerance(),
                        (d) -> obj.setTolerance(obj.getPositionTolerance(), d)),
                new TunableDouble("tunables/velocityConstraint",
                        obj.getConstraints().maxVelocity,
                        (d) -> obj.setConstraints(
                                new TrapezoidProfile.Constraints(d, obj.getConstraints().maxAcceleration))),
                new TunableDouble("tunables/accelerationConstraint",
                        obj.getConstraints().maxAcceleration,
                        (d) -> obj.setConstraints(
                                new TrapezoidProfile.Constraints(obj.getConstraints().maxVelocity, d))),
        };
    }

    @LogProfile(DCMotorSim.class)
    public static LogItem<?>[] logDCMotorSim(DCMotorSim obj) {
        return new LogItem<?>[] {
                new DoubleLogItem("angularPositionRad", obj::getAngularPositionRad, LogType.NT),
                new DoubleLogItem("angularPositionRotations", obj::getAngularPositionRotations, LogType.NT),
                new DoubleLogItem("angularVelocityRadPerSec", obj::getAngularVelocityRadPerSec, LogType.NT),
                new DoubleLogItem("angularVelocityRPM", obj::getAngularVelocityRPM, LogType.NT),
                new DoubleLogItem("currentDrawAmps", obj::getCurrentDrawAmps, LogType.NT),
        };
    }

    @LogProfile(SingleJointedArmSim.class)
    public static LogItem<?>[] logSingleJointedArmSim(SingleJointedArmSim obj) {
        return new LogItem<?>[] {
                new DoubleLogItem("angleRad", obj::getAngleRads, LogType.NT),
                new DoubleLogItem("velocityRadPerSec", obj::getVelocityRadPerSec, LogType.NT),
                new DoubleLogItem("currentDrawAmps", obj::getCurrentDrawAmps, LogType.NT),
        };
    }

    @LogProfile(RobotController.class)
    public static LogItem<?>[] logRobotController() {
        return new LogItem<?>[] {
                new DoubleLogItem("batteryVoltage", RobotController::getBatteryVoltage, LogType.NT),
                new BooleanLogItem("isBrownedOut", RobotController::isBrownedOut, LogType.NT),
                new IntegerLogItem("faultCount3v3", RobotController::getFaultCount3V3, LogType.NT),
                new IntegerLogItem("faultCount5v", RobotController::getFaultCount5V, LogType.NT),
                new IntegerLogItem("faultCount6v", RobotController::getFaultCount6V, LogType.NT),
                new DoubleLogItem("cpuTemp", RobotController::getCPUTemp, LogType.NT),
                new IntegerArrayLogItem("canBus", () -> {
                    // getCANStatus should only be called once per loop because it is expensive
                    CANStatus status = RobotController.getCANStatus();
                    return new int[] {
                            (int) (status.percentBusUtilization * 100),
                            status.busOffCount,
                            status.txFullCount,
                            status.receiveErrorCount,
                            status.transmitErrorCount
                    };
                }, LogType.NT),
        };
    }

}