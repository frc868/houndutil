package com.techhounds.houndutil.houndlog;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.studica.frc.AHRS;
import com.techhounds.houndutil.houndlog.annotations.LogProfile;
import com.techhounds.houndutil.houndlog.loggers.BooleanLogItem;
import com.techhounds.houndutil.houndlog.loggers.DoubleArrayLogItem;
import com.techhounds.houndutil.houndlog.loggers.DoubleLogItem;
import com.techhounds.houndutil.houndlog.loggers.FloatLogItem;
import com.techhounds.houndutil.houndlog.loggers.IntegerLogItem;
import com.techhounds.houndutil.houndlog.loggers.LogItem;
import com.techhounds.houndutil.houndlog.loggers.TunableDouble;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * The default set of log profiles that comes with HoundLog. Covers the devices
 * from the majority of FRC vendors, and logs the most important data from each
 * to NT/DataLog, depending on if they need to be streamed.
 */
public class LogProfiles {
    private static Map<Class<?>, String> ctreDeviceUnitNames = Map.of(
            ImmutableAngle.class, "Rotations",
            ImmutableAngularVelocity.class, "RotationsPerSecond",
            ImmutableAngularAcceleration.class, "RotationsPerSecondPerSecond",
            ImmutableTemperature.class, "DegreesCelsius",
            ImmutableVoltage.class, "",
            ImmutableCurrent.class, "Amps",
            Double.class, "");
    private static Map<Class<?>, String> siUnitNames = Map.of(
            ImmutableAngle.class, "Radians",
            ImmutableAngularVelocity.class, "RadiansPerSecond",
            ImmutableAngularAcceleration.class, "RadiansPerSecondPerSecond",
            ImmutableDistance.class, "Meters",
            ImmutableLinearVelocity.class, "MetersPerSecond",
            ImmutableLinearAcceleration.class, "MetersPerSecondPerSecond",
            ImmutableTemperature.class, "DegreesCelsius",
            ImmutableVoltage.class, "",
            ImmutableCurrent.class, "Amps",
            ImmutableDimensionless.class, "Value");

    /**
     * Builds TalonFX log items.
     * 
     * @param supplier the TalonFX object to use
     * @return the array of LogItems
     */
    @LogProfile(TalonFX.class)
    public static LogItem<?>[] logTalonFX(Supplier<TalonFX> supplier) {
        TalonFX obj = supplier.get(); // set obj once bc it doesn't get replaced

        List<StatusSignal<?>> signals = List.of(
                obj.getPosition(),
                obj.getVelocity(),
                obj.getAcceleration(),
                obj.getDeviceTemp(),
                obj.getMotorVoltage(),
                obj.getTorqueCurrent(),
                obj.getSupplyCurrent(),
                obj.getClosedLoopReference(),
                obj.getClosedLoopReferenceSlope(),
                obj.getClosedLoopError(),
                obj.getClosedLoopOutput(),
                obj.getClosedLoopFeedForward(),
                obj.getClosedLoopProportionalOutput(),
                obj.getClosedLoopIntegratedOutput(),
                obj.getClosedLoopDerivativeOutput());

        List<LogItem<?>> logItems = new ArrayList<>();

        signals.forEach((s) -> {
            SignalManager.register(obj.getNetwork().getName(), s);
            String name = s.getName();
            logItems.add(new DoubleLogItem(
                    // lower case first letter of signal name, append units from either map or
                    // default units
                    Character.toLowerCase(name.charAt(0)) + name.substring(1) +
                            ctreDeviceUnitNames.getOrDefault(s.getValue().getClass(), s.getUnits()),
                    s::getValueAsDouble, LogType.NT));
        });

        FaultLogger.register(obj);

        return logItems.toArray(new LogItem<?>[0]);
    }

    @LogProfile(DynamicMotionMagicVoltage.class)
    public static LogItem<?>[] logDynamicMotionMagicVoltage(Supplier<DynamicMotionMagicVoltage> supplier) {
        DynamicMotionMagicVoltage obj = supplier.get();

        return new LogItem<?>[] {
                new IntegerLogItem("slot", () -> obj.Slot, LogType.NT),
                new DoubleLogItem("feedForwardVoltage", () -> obj.getFeedForwardMeasure().in(Volts), LogType.NT),
                new DoubleLogItem("positionRotations", () -> obj.getPositionMeasure().in(Rotations), LogType.NT),
                new DoubleLogItem("velocityRotationsPerSecond", () -> obj.getVelocityMeasure().in(RotationsPerSecond),
                        LogType.NT),
        };
    }

    @LogProfile(MotionMagicVoltage.class)
    public static LogItem<?>[] logMotionMagicVoltage(Supplier<MotionMagicVoltage> supplier) {
        MotionMagicVoltage obj = supplier.get();

        return new LogItem<?>[] {
                new IntegerLogItem("slot", () -> obj.Slot, LogType.NT),
                new DoubleLogItem("feedForwardVoltage", () -> obj.getFeedForwardMeasure().in(Volts), LogType.NT),
                new DoubleLogItem("positionRotations", () -> obj.getPositionMeasure().in(Rotations), LogType.NT),
        };
    }

    @LogProfile(MotionMagicVelocityVoltage.class)
    public static LogItem<?>[] logMotionMagicVelocityVoltage(Supplier<MotionMagicVelocityVoltage> supplier) {
        MotionMagicVelocityVoltage obj = supplier.get();

        return new LogItem<?>[] {
                new IntegerLogItem("slot", () -> obj.Slot, LogType.NT),
                new DoubleLogItem("feedForwardVoltage", () -> obj.getFeedForwardMeasure().in(Volts), LogType.NT),
                new DoubleLogItem("velocityRotationsPerSecond", () -> obj.getVelocityMeasure().in(RotationsPerSecond),
                        LogType.NT)
        };
    }

    @LogProfile(VoltageOut.class)
    public static LogItem<?>[] logVoltageOut(Supplier<VoltageOut> supplier) {
        VoltageOut obj = supplier.get();

        return new LogItem<?>[] {
                new DoubleLogItem("outputVoltage", () -> obj.getOutputMeasure().in(Volts), LogType.NT),
        };
    }

    /**
     * Builds CANcoder log items.
     * 
     * @param supplier the CANcoder object to use
     * @return the array of LogItems
     */
    @LogProfile(CANcoder.class)
    public static LogItem<?>[] logCANcoder(Supplier<CANcoder> supplier) {
        CANcoder obj = supplier.get(); // set obj once bc it doesn't get replaced

        List<StatusSignal<?>> signals = List.of(
                obj.getAbsolutePosition(),
                obj.getPosition(),
                obj.getVelocity());

        List<LogItem<?>> logItems = new ArrayList<>();

        signals.forEach((s) -> {
            SignalManager.register(obj.getNetwork().getName(), s);
            String name = s.getName();
            logItems.add(new DoubleLogItem(
                    // lower case first letter of signal name, append units from either map or
                    // default units
                    Character.toLowerCase(name.charAt(0)) + name.substring(1) +
                            ctreDeviceUnitNames.getOrDefault(s.getValue().getClass(), s.getUnits()),
                    s::getValueAsDouble, LogType.NT));
        });

        FaultLogger.register(obj);

        return logItems.toArray(new LogItem<?>[0]);
    }

    /**
     * Builds NavX log items.
     * 
     * @param supplier the navx to use
     * @return the array of LogItems
     */
    @LogProfile(AHRS.class)
    public static LogItem<?>[] logNavX(Supplier<AHRS> supplier) {
        AHRS obj = (AHRS) supplier.get(); // set obj once bc it doesn't get replaced

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
     * @param supplier the navx to use
     * @return the array of LogItems
     */
    @LogProfile(Pigeon2.class)
    public static LogItem<?>[] logPigeon2(Supplier<Pigeon2> supplier) {
        Pigeon2 obj = supplier.get(); // set obj once bc it doesn't get replaced

        List<StatusSignal<?>> signals = List.of(
                obj.getPitch(),
                obj.getRoll(),
                obj.getYaw());

        List<LogItem<?>> logItems = new ArrayList<>();

        signals.forEach((s) -> {
            SignalManager.register(obj.getNetwork().getName(), s);
            String name = s.getName();
            logItems.add(new DoubleLogItem(
                    // lower case first letter of signal name, append units from either map or
                    // default units
                    Character.toLowerCase(name.charAt(0)) + name.substring(1) +
                            ctreDeviceUnitNames.getOrDefault(s.getValue().getClass(), s.getUnits()),
                    s::getValueAsDouble, LogType.NT));
        });

        FaultLogger.register(obj);

        return logItems.toArray(new LogItem<?>[0]);
    }

    /**
     * Builds PDH log items.
     * 
     * @param supplier the PDH instance to use
     * @return the array of LogItems
     */
    @LogProfile(PowerDistribution.class)
    public static LogItem<?>[] logPDH(Supplier<PowerDistribution> supplier) {
        PowerDistribution obj = (PowerDistribution) supplier.get(); // set obj once bc it doesn't get replaced

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
     * Builds PIDController log items.
     * 
     * @param supplier the {@link PIDController} to use
     * @return the array of LogItems
     */
    @LogProfile(PIDController.class)
    public static LogItem<?>[] logPIDController(Supplier<PIDController> supplier) {
        PIDController obj = (PIDController) supplier.get(); // set obj once bc it doesn't get replaced

        return new LogItem<?>[] {
                new DoubleLogItem("setpoint", () -> obj.getSetpoint(),
                        LogType.NT),
                new BooleanLogItem("atSetpoint", () -> obj.atSetpoint(),
                        LogType.NT),
                new DoubleLogItem("error", () -> obj.getError(), LogType.NT),
                new TunableDouble("tunables/kP", obj.getP(), (d) -> obj.setP(d)),
                new TunableDouble("tunables/kI", obj.getI(), (d) -> obj.setI(d)),
                new TunableDouble("tunables/kD", obj.getD(), (d) -> obj.setD(d)),
                new TunableDouble("tunables/errorTolerance", obj.getErrorTolerance(),
                        (d) -> obj.setTolerance(d)),
                new TunableDouble("tunables/errorDerivativeTolerance", obj.getErrorDerivativeTolerance(),
                        (d) -> obj.setTolerance(obj.getErrorTolerance(), d)),
        };
    }

    /**
     * Builds ProfiledPIDController log items.
     * 
     * @param supplier the {@link ProfiledPIDController} to use
     * @return the array of LogItems
     */
    @LogProfile(ProfiledPIDController.class)
    public static LogItem<?>[] logProfiledPIDController(Supplier<ProfiledPIDController> supplier) {
        ProfiledPIDController obj = (ProfiledPIDController) supplier.get(); // set obj once bc it doesn't get replaced

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

    /**
     * Builds DCMotorSim log items.
     * 
     * @param supplier the {@link DCMotorSim} to use
     * @return the array of LogItems
     */
    @LogProfile(DCMotorSim.class)
    public static LogItem<?>[] logDCMotorSim(Supplier<DCMotorSim> supplier) {
        DCMotorSim obj = (DCMotorSim) supplier.get(); // set obj once bc it doesn't get replaced

        return new LogItem<?>[] {
                new DoubleLogItem("angularPositionRad", obj::getAngularPositionRad, LogType.NT),
                new DoubleLogItem("angularPositionRotations", obj::getAngularPositionRotations, LogType.NT),
                new DoubleLogItem("angularVelocityRadPerSec", obj::getAngularVelocityRadPerSec, LogType.NT),
                new DoubleLogItem("angularVelocityRPM", obj::getAngularVelocityRPM, LogType.NT),
                new DoubleLogItem("currentDrawAmps", obj::getCurrentDrawAmps, LogType.NT),
        };
    }

    /**
     * Builds ElevatorSim log items.
     * 
     * @param supplier the ElevatorSim object to use
     * @return the array of LogItems
     */
    @LogProfile(ElevatorSim.class)
    public static LogItem<?>[] logElevatorSim(Supplier<ElevatorSim> supplier) {
        ElevatorSim obj = (ElevatorSim) supplier.get(); // set obj once bc it doesn't get replaced

        return new LogItem<?>[] {
                new DoubleLogItem("positionMeters", () -> obj.getPositionMeters(), LogType.NT),
                new DoubleLogItem("velocityMetersPerSecond", () -> obj.getVelocityMetersPerSecond(), LogType.NT),
                new DoubleLogItem("currentDrawAmps", () -> obj.getCurrentDrawAmps(), LogType.NT),
        };
    }

    /**
     * Builds SingleJointedArmSim log items.
     * 
     * @param supplier the {@link SingleJointedArmSim} to use
     * @return the array of LogItems
     */
    @LogProfile(SingleJointedArmSim.class)
    public static LogItem<?>[] logSingleJointedArmSim(Supplier<SingleJointedArmSim> supplier) {
        SingleJointedArmSim obj = (SingleJointedArmSim) supplier.get(); // set obj once bc it doesn't get replaced

        return new LogItem<?>[] {
                new DoubleLogItem("angleRad", obj::getAngleRads, LogType.NT),
                new DoubleLogItem("velocityRadPerSec", obj::getVelocityRadPerSec, LogType.NT),
                new DoubleLogItem("currentDrawAmps", obj::getCurrentDrawAmps, LogType.NT),
        };
    }

    /**
     * Builds RobotController log items. This must be called manually with the
     * creation of a log group.
     * 
     * @return the array of LogItems
     */
    public static LogItem<?>[] logRobotController() {
        return new LogItem<?>[] {
                new DoubleLogItem("batteryVoltage", RobotController::getBatteryVoltage, LogType.NT),
                new BooleanLogItem("isBrownedOut", RobotController::isBrownedOut, LogType.NT),
                new IntegerLogItem("faultCount3v3", RobotController::getFaultCount3V3, LogType.NT),
                new IntegerLogItem("faultCount5v", RobotController::getFaultCount5V, LogType.NT),
                new IntegerLogItem("faultCount6v", RobotController::getFaultCount6V, LogType.NT),
                new DoubleLogItem("cpuTemp", RobotController::getCPUTemp, LogType.NT),
                // new IntegerArrayLogItem("canBus", () -> {
                // // getCANStatus should only be called once per loop because it is expensive
                // CANStatus status = RobotController.getCANStatus();
                // return new int[] {
                // (int) (status.percentBusUtilization * 100),
                // status.busOffCount,
                // status.txFullCount,
                // status.receiveErrorCount,
                // status.transmitErrorCount
                // };
                // }, LogType.NT),
        };
    }

    /**
     * Builds Measure log items.
     * 
     * @param supplier the supplier of {@link ImmutableAngle} to use
     * @return the array of LogItems
     */
    @LogProfile(Measure.class)
    public static LogItem<?>[] logMeasure(Supplier<Measure<?>> supplier) {
        String name = supplier.get().getClass().getSimpleName().replace("Immutable", "");
        return new LogItem<?>[] {
                // get the supplier every loop bc the objects get replaced
                new DoubleLogItem(
                        // lower case type, append unit from SI map or from Units default, removing
                        // spaces
                        Character.toLowerCase(name.charAt(0)) + name.substring(1)
                                + siUnitNames.getOrDefault(supplier.get().getClass(),
                                        supplier.get().baseUnit().name().replace(" ", "")),
                        () -> supplier.get().baseUnitMagnitude(), LogType.NT)
        };
    }

    // TODO is there a way to do a general method for any Measure?
    // TODO log canivore?
}
