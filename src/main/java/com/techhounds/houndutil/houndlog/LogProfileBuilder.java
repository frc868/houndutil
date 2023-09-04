package com.techhounds.houndutil.houndlog;

import java.lang.reflect.Field;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

import com.techhounds.houndutil.houndlog.enums.LogType;
import com.techhounds.houndutil.houndlog.logitems.AbstractLogItem;
import com.techhounds.houndutil.houndlog.logitems.BooleanLogItem;
import com.techhounds.houndutil.houndlog.logitems.DoubleLogItem;
import com.techhounds.houndutil.houndlog.logitems.FloatLogItem;
import com.techhounds.houndutil.houndlog.logitems.IntegerLogItem;
import com.techhounds.houndutil.houndlog.logitems.StringLogItem;
import com.techhounds.houndutil.houndlog.logitems.TunableDouble;

/**
 * A helper class that will automatically create an array of {@link LogItem}s
 * based on the type of object. Logs useful information about each device.
 * Reduces verbosity by a TON.
 * 
 * @author dr
 */
public class LogProfileBuilder {

    /**
     * Builds CANSparkMax log items.
     * 
     * @param obj the CANSparkMax object to use
     * @return the array of LogItems
     */
    public static AbstractLogItem<?>[] buildCANSparkMaxLogItems(CANSparkMax obj) {
        return new AbstractLogItem<?>[] {
                new DoubleLogItem("Encoder Distance", obj.getEncoder()::getPosition, LogType.NT),
                new DoubleLogItem("Encoder Distance Conversion Factor",
                        obj.getEncoder()::getPositionConversionFactor, LogType.CONSOLE),
                new DoubleLogItem("Encoder Speed", obj.getEncoder()::getVelocity, LogType.NT),
                new DoubleLogItem("Encoder Speed Conversion Factor",
                        obj.getEncoder()::getVelocityConversionFactor, LogType.CONSOLE),
                new DoubleLogItem("Closed Loop Ramp Rate", obj::getClosedLoopRampRate,
                        LogType.CONSOLE),
                new DoubleLogItem("Open Loop Ramp Rate", obj::getOpenLoopRampRate, LogType.CONSOLE),
                new DoubleLogItem("Speed", obj::get, LogType.NT),
                new DoubleLogItem("Applied Output", obj::getAppliedOutput, LogType.NT),
                new DoubleLogItem("Bus Voltage", obj::getBusVoltage, LogType.CONSOLE),
                new DoubleLogItem("Motor Temperature", obj::getMotorTemperature, LogType.NT),
                new DoubleLogItem("Output Current", obj::getOutputCurrent, LogType.NT),
                new IntegerLogItem("Device ID", obj::getDeviceId, LogType.CONSOLE),
                new StringLogItem("Firmware Version", obj::getFirmwareString, LogType.CONSOLE),
                new BooleanLogItem("Is In Brake Mode",
                        () -> obj.getIdleMode() == CANSparkMax.IdleMode.kBrake, LogType.CONSOLE),
                new BooleanLogItem("Is Inverted", obj::getInverted, LogType.CONSOLE),
                new BooleanLogItem("Is Follower", obj::isFollower, LogType.CONSOLE),
                new BooleanLogItem("Faults/Brownout",
                        () -> obj.getFault(CANSparkMax.FaultID.kBrownout), LogType.NT),
                new BooleanLogItem("Faults/Has Reset",
                        () -> obj.getFault(CANSparkMax.FaultID.kHasReset), LogType.DATALOG),
                new BooleanLogItem("Faults/Motor Fault",
                        () -> obj.getFault(CANSparkMax.FaultID.kMotorFault), LogType.DATALOG),
                new BooleanLogItem("Faults/Other Fault",
                        () -> obj.getFault(CANSparkMax.FaultID.kOtherFault), LogType.DATALOG),
                new BooleanLogItem("Faults/Overcurrent",
                        () -> obj.getFault(CANSparkMax.FaultID.kOvercurrent), LogType.DATALOG),
                new BooleanLogItem("Faults/Sensor Fault",
                        () -> obj.getFault(CANSparkMax.FaultID.kSensorFault), LogType.DATALOG),
                new BooleanLogItem("Faults/Stalled", () -> obj.getFault(CANSparkMax.FaultID.kStall),
                        LogType.DATALOG),
                new BooleanLogItem("Faults/DRV Fault",
                        () -> obj.getFault(CANSparkMax.FaultID.kStall), LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/Brownout",
                        () -> obj.getStickyFault(CANSparkMax.FaultID.kBrownout), LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/Has Reset",
                        () -> obj.getStickyFault(CANSparkMax.FaultID.kHasReset), LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/Motor Fault",
                        () -> obj.getStickyFault(CANSparkMax.FaultID.kMotorFault),
                        LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/Other Fault",
                        () -> obj.getStickyFault(CANSparkMax.FaultID.kOtherFault),
                        LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/Overcurrent",
                        () -> obj.getStickyFault(CANSparkMax.FaultID.kOvercurrent),
                        LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/Sensor Fault",
                        () -> obj.getStickyFault(CANSparkMax.FaultID.kSensorFault),
                        LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/Stalled",
                        () -> obj.getStickyFault(CANSparkMax.FaultID.kStall), LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/DRV Fault",
                        () -> obj.getStickyFault(CANSparkMax.FaultID.kStall), LogType.DATALOG),
        };
    }

    /**
     * Builds CANSparkMax log items.
     * 
     * @param obj the CANSparkMax object to use
     * @return the array of LogItems
     */
    public static AbstractLogItem<?>[] buildCANCoderLogItems(CANCoder obj) {
        return new AbstractLogItem<?>[] {
                new DoubleLogItem("Absolute Position", obj::getAbsolutePosition, LogType.NT),
                new DoubleLogItem("Position", obj::getPosition, LogType.NT),
                new DoubleLogItem("Velocity", obj::getVelocity, LogType.NT),
                new DoubleLogItem("Bus Voltage", obj::getBusVoltage, LogType.DATALOG),
                new IntegerLogItem("Device ID", obj::getDeviceID, LogType.DATALOG),
                new StringLogItem("Magnetic Field Strength", obj.getMagnetFieldStrength()::name,
                        LogType.DATALOG),
        };
    }

    /**
     * Builds CANSparkMax log items.
     * 
     * @param obj the CANSparkMax object to use
     * @return the array of LogItems
     */
    public static AbstractLogItem<?>[] buildSparkMaxAbsoluteEncoderLogItems(SparkMaxAbsoluteEncoder obj) {
        return new AbstractLogItem<?>[] {
                new DoubleLogItem("Position", obj::getPosition, LogType.NT),
                new DoubleLogItem("Velocity", obj::getVelocity, LogType.NT),
                new DoubleLogItem("Zero Offset", obj::getZeroOffset, LogType.NT),
                new BooleanLogItem("Is Inverted", obj::getInverted, LogType.DATALOG),
                new DoubleLogItem("Position Conversion Factor", obj::getPositionConversionFactor, LogType.DATALOG),
                new DoubleLogItem("Velocity Conversion Factor", obj::getVelocityConversionFactor, LogType.DATALOG),
        };
    }

    /**
     * Builds navX log items.
     * 
     * @param obj the navx to use
     * @return the array of LogItems
     */
    public static AbstractLogItem<?>[] buildNavXLogItems(AHRS obj) {
        return new AbstractLogItem<?>[] {
                new FloatLogItem("Pitch", obj::getPitch, LogType.NT),
                new FloatLogItem("Roll", obj::getRoll, LogType.NT),
                new FloatLogItem("Yaw", obj::getYaw, LogType.NT),
                new DoubleLogItem("Angle Rotation Rate", obj::getRate, LogType.DATALOG),
                new FloatLogItem("X Axis Acceleration", obj::getWorldLinearAccelX, LogType.NT),
                new FloatLogItem("Y Axis Acceleration", obj::getWorldLinearAccelY, LogType.NT),
                new FloatLogItem("Z Axis Acceleration", obj::getWorldLinearAccelZ, LogType.NT),
                new FloatLogItem("Compass Heading", obj::getCompassHeading, LogType.NT),
                new BooleanLogItem("Is Calibrating", obj::isCalibrating, LogType.NT),
                new BooleanLogItem("Is Magnetometer Calibrated", obj::isMagnetometerCalibrated,
                        LogType.NT),
                new BooleanLogItem("Is Connected", obj::isConnected, LogType.DATALOG),
                new BooleanLogItem("Is Moving", obj::isMoving, LogType.NT),
                new BooleanLogItem("Is Rotating", obj::isRotating, LogType.NT),
                new BooleanLogItem("Is Magnetic Disturbance", obj::isMagneticDisturbance,
                        LogType.DATALOG),
                new FloatLogItem("Temperature", obj::getTempC, LogType.NT),
                new DoubleLogItem("Update Count", obj::getUpdateCount, LogType.CONSOLE),
                new StringLogItem("Firmware Version", obj::getFirmwareVersion, LogType.CONSOLE)

        };
    }

    /**
     * Builds Pigeon 2 log items.
     * 
     * @param obj the navx to use
     * @return the array of LogItems
     */
    public static AbstractLogItem<?>[] buildPigeon2LogItems(Pigeon2 obj) {
        return new AbstractLogItem<?>[] {
                new DoubleLogItem("Pitch", obj::getPitch, LogType.NT),
                new DoubleLogItem("Roll", obj::getRoll, LogType.NT),
                new DoubleLogItem("Yaw", obj::getYaw, LogType.NT),
                new DoubleLogItem("YawRad", () -> Units.degreesToRadians(obj.getYaw()), LogType.NT),
                new DoubleLogItem("Absolute Compass Heading", obj::getAbsoluteCompassHeading,
                        LogType.DATALOG),
                new IntegerLogItem("Uptime", obj::getUpTime, LogType.DATALOG),
                new DoubleLogItem("Temperature", obj::getTemp, LogType.DATALOG),
                new StringLogItem("Last Error", obj.getLastError()::toString, LogType.CONSOLE),
                new IntegerLogItem("Device ID", obj::getDeviceID, LogType.CONSOLE),
                new IntegerLogItem("Firmware Version", obj::getFirmwareVersion, LogType.DATALOG),
        };
    }

    /**
     * Builds DoubleSolenoid log items.
     * 
     * @param obj the {@link DoubleSolenoid} to use
     * @return the array of LogItems
     */
    public static AbstractLogItem<?>[] buildDoubleSolenoidLogItems(DoubleSolenoid obj) {
        return new AbstractLogItem<?>[] {
                new BooleanLogItem("Position", () -> obj.get() == DoubleSolenoid.Value.kForward,
                        LogType.NT),
                new DoubleLogItem("Forward Channel", () -> (double) obj.getFwdChannel(),
                        LogType.CONSOLE),
                new DoubleLogItem("Reverse Channel", () -> (double) obj.getRevChannel(),
                        LogType.CONSOLE),
                new BooleanLogItem("Is Forward Solenoid Disabled", obj::isFwdSolenoidDisabled,
                        LogType.DATALOG),
                new BooleanLogItem("Is Reverse Solenoid Disabled", obj::isRevSolenoidDisabled,
                        LogType.DATALOG)
        };
    }

    /**
     * Builds PDH log items.
     * 
     * @param obj the PDH instance to use
     * @return the array of LogItems
     */
    public static AbstractLogItem<?>[] buildPDHLogItems(PowerDistribution obj) {
        return new AbstractLogItem<?>[] {
                new DoubleLogItem("Voltage", obj::getVoltage, LogType.NT),
                new DoubleLogItem("Temperature", obj::getTemperature, LogType.NT),
                new DoubleLogItem("Total Current (A)", obj::getTotalCurrent, LogType.NT),
                new DoubleLogItem("Total Power (W)", obj::getTotalPower, LogType.NT),
                new DoubleLogItem("Total Energy (J)", obj::getTotalEnergy, LogType.NT),
                new DoubleLogItem("Channel 0 Current", () -> obj.getCurrent(0), LogType.DATALOG),
                new DoubleLogItem("Channel 1 Current", () -> obj.getCurrent(1), LogType.DATALOG),
                new DoubleLogItem("Channel 2 Current", () -> obj.getCurrent(2), LogType.DATALOG),
                new DoubleLogItem("Channel 3 Current", () -> obj.getCurrent(3), LogType.DATALOG),
                new DoubleLogItem("Channel 4 Current", () -> obj.getCurrent(4), LogType.DATALOG),
                new DoubleLogItem("Channel 5 Current", () -> obj.getCurrent(5), LogType.DATALOG),
                new DoubleLogItem("Channel 6 Current", () -> obj.getCurrent(6), LogType.DATALOG),
                new DoubleLogItem("Channel 7 Current", () -> obj.getCurrent(7), LogType.DATALOG),
                new DoubleLogItem("Channel 8 Current", () -> obj.getCurrent(8), LogType.DATALOG),
                new DoubleLogItem("Channel 9 Current", () -> obj.getCurrent(9), LogType.DATALOG),
                new DoubleLogItem("Channel 10 Current", () -> obj.getCurrent(10), LogType.DATALOG),
                new DoubleLogItem("Channel 11 Current", () -> obj.getCurrent(11), LogType.DATALOG),
                new DoubleLogItem("Channel 12 Current", () -> obj.getCurrent(12), LogType.DATALOG),
                new DoubleLogItem("Channel 13 Current", () -> obj.getCurrent(13), LogType.DATALOG),
                new DoubleLogItem("Channel 14 Current", () -> obj.getCurrent(14), LogType.DATALOG),
                new DoubleLogItem("Channel 15 Current", () -> obj.getCurrent(15), LogType.DATALOG),
                new DoubleLogItem("Channel 16 Current", () -> obj.getCurrent(16), LogType.DATALOG),
                new DoubleLogItem("Channel 17 Current", () -> obj.getCurrent(17), LogType.DATALOG),
                new DoubleLogItem("Channel 18 Current", () -> obj.getCurrent(18), LogType.DATALOG),
                new DoubleLogItem("Channel 19 Current", () -> obj.getCurrent(19), LogType.DATALOG),
                new DoubleLogItem("Channel 20 Current", () -> obj.getCurrent(20), LogType.DATALOG),
                new DoubleLogItem("Channel 21 Current", () -> obj.getCurrent(21), LogType.DATALOG),
                new DoubleLogItem("Channel 22 Current", () -> obj.getCurrent(22), LogType.DATALOG),
                new DoubleLogItem("Channel 23 Current", () -> obj.getCurrent(23), LogType.DATALOG),
                new DoubleLogItem("Version/Firmware Major",
                        () -> (double) obj.getVersion().firmwareMajor, LogType.CONSOLE),
                new DoubleLogItem("Version/Firmware Minor",
                        () -> (double) obj.getVersion().firmwareMinor, LogType.CONSOLE),
                new DoubleLogItem("Version/Firmware Fix",
                        () -> (double) obj.getVersion().firmwareFix, LogType.CONSOLE),
                new DoubleLogItem("Version/Hardware Major",
                        () -> (double) obj.getVersion().hardwareMajor, LogType.CONSOLE),
                new DoubleLogItem("Version/Hardware Minor",
                        () -> (double) obj.getVersion().hardwareMinor, LogType.CONSOLE),
                new DoubleLogItem("Unique ID", () -> (double) obj.getVersion().uniqueId,
                        LogType.CONSOLE),
                new StringLogItem("Type",
                        () -> obj.getType() == ModuleType.kRev ? "Rev PDH" : "CTRE PDP",
                        LogType.DATALOG),
                new BooleanLogItem("Is Switchable Channel On", obj::getSwitchableChannel,
                        LogType.CONSOLE),
                new BooleanLogItem("Faults/Channel 0 Breaker Fault",
                        () -> obj.getFaults().Channel0BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Faults/Channel 1 Breaker Fault",
                        () -> obj.getFaults().Channel1BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Faults/Channel 2 Breaker Fault",
                        () -> obj.getFaults().Channel2BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Faults/Channel 3 Breaker Fault",
                        () -> obj.getFaults().Channel3BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Faults/Channel 4 Breaker Fault",
                        () -> obj.getFaults().Channel4BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Faults/Channel 5 Breaker Fault",
                        () -> obj.getFaults().Channel5BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Faults/Channel 6 Breaker Fault",
                        () -> obj.getFaults().Channel6BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Faults/Channel 7 Breaker Fault",
                        () -> obj.getFaults().Channel7BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Faults/Channel 8 Breaker Fault",
                        () -> obj.getFaults().Channel8BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Faults/Channel 9 Breaker Fault",
                        () -> obj.getFaults().Channel9BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Faults/Channel 10 Breaker Fault",
                        () -> obj.getFaults().Channel11BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Faults/Channel 11 Breaker Fault",
                        () -> obj.getFaults().Channel10BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Faults/Channel 12 Breaker Fault",
                        () -> obj.getFaults().Channel12BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Faults/Channel 13 Breaker Fault",
                        () -> obj.getFaults().Channel13BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Faults/Channel 14 Breaker Fault",
                        () -> obj.getFaults().Channel14BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Faults/Channel 15 Breaker Fault",
                        () -> obj.getFaults().Channel15BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Faults/Channel 16 Breaker Fault",
                        () -> obj.getFaults().Channel16BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Faults/Channel 17 Breaker Fault",
                        () -> obj.getFaults().Channel17BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Faults/Channel 18 Breaker Fault",
                        () -> obj.getFaults().Channel18BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Faults/Channel 19 Breaker Fault",
                        () -> obj.getFaults().Channel19BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Faults/Channel 20 Breaker Fault",
                        () -> obj.getFaults().Channel20BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Faults/Channel 21 Breaker Fault",
                        () -> obj.getFaults().Channel21BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Faults/Channel 22 Breaker Fault",
                        () -> obj.getFaults().Channel22BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Faults/Channel 23 Breaker Fault",
                        () -> obj.getFaults().Channel23BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Faults/Brownout", () -> obj.getFaults().Brownout, LogType.DATALOG),
                new BooleanLogItem("Faults/CAN Warning",
                        () -> obj.getFaults().CanWarning, LogType.DATALOG),
                new BooleanLogItem("Faults/Hardware Fault", () -> obj.getFaults().HardwareFault,
                        LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/Channel 0 Breaker Fault",
                        () -> obj.getStickyFaults().Channel0BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/Channel 1 Breaker Fault",
                        () -> obj.getStickyFaults().Channel1BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/Channel 2 Breaker Fault",
                        () -> obj.getStickyFaults().Channel2BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/Channel 3 Breaker Fault",
                        () -> obj.getStickyFaults().Channel3BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/Channel 4 Breaker Fault",
                        () -> obj.getStickyFaults().Channel4BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/Channel 5 Breaker Fault",
                        () -> obj.getStickyFaults().Channel5BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/Channel 6 Breaker Fault",
                        () -> obj.getStickyFaults().Channel6BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/Channel 7 Breaker Fault",
                        () -> obj.getStickyFaults().Channel7BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/Channel 8 Breaker Fault",
                        () -> obj.getStickyFaults().Channel8BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/Channel 9 Breaker Fault",
                        () -> obj.getStickyFaults().Channel9BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/Channel 10 Breaker Fault",
                        () -> obj.getStickyFaults().Channel11BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/Channel 11 Breaker Fault",
                        () -> obj.getStickyFaults().Channel10BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/Channel 12 Breaker Fault",
                        () -> obj.getStickyFaults().Channel12BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/Channel 13 Breaker Fault",
                        () -> obj.getStickyFaults().Channel13BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/Channel 14 Breaker Fault",
                        () -> obj.getStickyFaults().Channel14BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/Channel 15 Breaker Fault",
                        () -> obj.getStickyFaults().Channel15BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/Channel 16 Breaker Fault",
                        () -> obj.getStickyFaults().Channel16BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/Channel 17 Breaker Fault",
                        () -> obj.getStickyFaults().Channel17BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/Channel 18 Breaker Fault",
                        () -> obj.getStickyFaults().Channel18BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/Channel 19 Breaker Fault",
                        () -> obj.getStickyFaults().Channel19BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/Channel 20 Breaker Fault",
                        () -> obj.getStickyFaults().Channel20BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/Channel 21 Breaker Fault",
                        () -> obj.getStickyFaults().Channel21BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/Channel 22 Breaker Fault",
                        () -> obj.getStickyFaults().Channel22BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/Channel 23 Breaker Fault",
                        () -> obj.getStickyFaults().Channel23BreakerFault, LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/Brownout", () -> obj.getStickyFaults().Brownout,
                        LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/CAN Bus Off",
                        () -> obj.getStickyFaults().CanBusOff, LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/CAN Warning",
                        () -> obj.getStickyFaults().CanWarning, LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/Has Reset", () -> obj.getStickyFaults().HasReset,
                        LogType.DATALOG)

        };
    }

    /**
     * Builds REV Pneumatics Hub log items.
     * 
     * @param obj the PH instance to use
     * @return the array of LogItems
     */
    public static AbstractLogItem<?>[] buildPneumaticHubLogItems(PneumaticHub obj) {
        return new AbstractLogItem<?>[] {
                new DoubleLogItem("Input Voltage", obj::getInputVoltage, LogType.NT),
                new DoubleLogItem("5V Regulated Voltage", obj::get5VRegulatedVoltage, LogType.NT),
                new DoubleLogItem("Module Number", () -> (double) obj.getModuleNumber(),
                        LogType.DATALOG),
                new DoubleLogItem("Channel 0 Pressure", () -> obj.getPressure(0), LogType.DATALOG),
                new DoubleLogItem("Channel 1 Pressure", () -> obj.getPressure(1), LogType.DATALOG),
                new DoubleLogItem("Channel 2 Pressure", () -> obj.getPressure(2), LogType.DATALOG),
                new DoubleLogItem("Channel 3 Pressure", () -> obj.getPressure(3), LogType.DATALOG),
                new DoubleLogItem("Channel 4 Pressure", () -> obj.getPressure(4), LogType.DATALOG),
                new DoubleLogItem("Channel 5 Pressure", () -> obj.getPressure(5), LogType.DATALOG),
                new DoubleLogItem("Channel 6 Pressure", () -> obj.getPressure(6), LogType.DATALOG),
                new DoubleLogItem("Channel 7 Pressure", () -> obj.getPressure(7), LogType.DATALOG),
                new DoubleLogItem("Channel 8 Pressure", () -> obj.getPressure(8), LogType.DATALOG),
                new DoubleLogItem("Channel 9 Pressure", () -> obj.getPressure(9), LogType.DATALOG),
                new DoubleLogItem("Channel 10 Pressure", () -> obj.getPressure(10), LogType.DATALOG),
                new DoubleLogItem("Channel 11 Pressure", () -> obj.getPressure(11), LogType.DATALOG),
                new DoubleLogItem("Channel 12 Pressure", () -> obj.getPressure(12), LogType.DATALOG),
                new DoubleLogItem("Channel 13 Pressure", () -> obj.getPressure(13), LogType.DATALOG),
                new DoubleLogItem("Channel 14 Pressure", () -> obj.getPressure(14), LogType.DATALOG),
                new DoubleLogItem("Channel 15 Pressure", () -> obj.getPressure(15), LogType.DATALOG),
                new BooleanLogItem("Pressure Switch", obj::getPressureSwitch, LogType.DATALOG),
                new DoubleLogItem("Compressor Current", obj::getCompressorCurrent, LogType.NT),
                new BooleanLogItem("Is Compressor Running", obj::getCompressor, LogType.NT),
                new DoubleLogItem("Solenoids", () -> (double) obj.getSolenoids(), LogType.DATALOG),
                new DoubleLogItem("Solenoids Total Current", obj::getSolenoidsTotalCurrent,
                        LogType.DATALOG),
                new DoubleLogItem("Solenoids Voltage", obj::getSolenoidsVoltage, LogType.DATALOG),
                new DoubleLogItem("Version/Firmware Major",
                        () -> (double) obj.getVersion().firmwareMajor, LogType.CONSOLE),
                new DoubleLogItem("Version/Firmware Minor",
                        () -> (double) obj.getVersion().firmwareMinor, LogType.CONSOLE),
                new DoubleLogItem("Version/Firmware Fix",
                        () -> (double) obj.getVersion().firmwareFix, LogType.CONSOLE),
                new DoubleLogItem("Version/Hardware Major",
                        () -> (double) obj.getVersion().hardwareMajor, LogType.CONSOLE),
                new DoubleLogItem("Version/Hardware Minor",
                        () -> (double) obj.getVersion().hardwareMinor, LogType.CONSOLE),
                new BooleanLogItem("Faults/Brownout", () -> obj.getFaults().Brownout, LogType.NT),
                new BooleanLogItem("Faults/CAN Warning", () -> obj.getFaults().CanWarning,
                        LogType.DATALOG),
                new BooleanLogItem("Faults/Channel 0 Fault", () -> obj.getFaults().Channel0Fault,
                        LogType.DATALOG),
                new BooleanLogItem("Faults/Channel 1 Fault", () -> obj.getFaults().Channel1Fault,
                        LogType.DATALOG),
                new BooleanLogItem("Faults/Channel 2 Fault", () -> obj.getFaults().Channel2Fault,
                        LogType.DATALOG),
                new BooleanLogItem("Faults/Channel 3 Fault", () -> obj.getFaults().Channel3Fault,
                        LogType.DATALOG),
                new BooleanLogItem("Faults/Channel 4 Fault", () -> obj.getFaults().Channel4Fault,
                        LogType.DATALOG),
                new BooleanLogItem("Faults/Channel 5 Fault", () -> obj.getFaults().Channel5Fault,
                        LogType.DATALOG),
                new BooleanLogItem("Faults/Channel 6 Fault", () -> obj.getFaults().Channel6Fault,
                        LogType.DATALOG),
                new BooleanLogItem("Faults/Channel 7 Fault", () -> obj.getFaults().Channel7Fault,
                        LogType.DATALOG),
                new BooleanLogItem("Faults/Channel 8 Fault", () -> obj.getFaults().Channel8Fault,
                        LogType.DATALOG),
                new BooleanLogItem("Faults/Channel 9 Fault", () -> obj.getFaults().Channel9Fault,
                        LogType.DATALOG),
                new BooleanLogItem("Faults/Channel 10 Fault", () -> obj.getFaults().Channel10Fault,
                        LogType.DATALOG),
                new BooleanLogItem("Faults/Channel 11 Fault", () -> obj.getFaults().Channel11Fault,
                        LogType.DATALOG),
                new BooleanLogItem("Faults/Channel 12 Fault", () -> obj.getFaults().Channel12Fault,
                        LogType.DATALOG),
                new BooleanLogItem("Faults/Channel 13 Fault", () -> obj.getFaults().Channel13Fault,
                        LogType.DATALOG),
                new BooleanLogItem("Faults/Channel 14 Fault", () -> obj.getFaults().Channel14Fault,
                        LogType.DATALOG),
                new BooleanLogItem("Faults/Channel 15 Fault", () -> obj.getFaults().Channel15Fault,
                        LogType.DATALOG),
                new BooleanLogItem("Faults/Compressor Open", () -> obj.getFaults().CompressorOpen,
                        LogType.NT),
                new BooleanLogItem("Faults/Compressor Over Current",
                        () -> obj.getFaults().CompressorOverCurrent, LogType.NT),
                new BooleanLogItem("Faults/Hardware Fault", () -> obj.getFaults().HardwareFault,
                        LogType.NT),
                new BooleanLogItem("Faults/Solenoid Over Current",
                        () -> obj.getFaults().SolenoidOverCurrent, LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/Brownout", () -> obj.getStickyFaults().Brownout,
                        LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/CAN Bus Off",
                        () -> obj.getStickyFaults().CanBusOff, LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/CAN Warning",
                        () -> obj.getStickyFaults().CanWarning, LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/Compressor Open",
                        () -> obj.getStickyFaults().CompressorOpen, LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/Compressor Over Current",
                        () -> obj.getStickyFaults().CompressorOverCurrent, LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/Has Reset", () -> obj.getStickyFaults().HasReset,
                        LogType.DATALOG),
                new BooleanLogItem("Sticky Faults/Solenoid Over Current",
                        () -> obj.getStickyFaults().SolenoidOverCurrent, LogType.DATALOG)

        };
    }

    /**
     * Builds PIDController log items.
     * 
     * @param obj the {@link PIDController} to use
     * @return the array of LogItems
     */
    public static AbstractLogItem<?>[] buildPIDControllerLogItems(PIDController obj) {
        return new AbstractLogItem<?>[] {
                new DoubleLogItem("Setpoint", () -> obj.getSetpoint(),
                        LogType.NT),
                new BooleanLogItem("At Setpoint", () -> obj.atSetpoint(),
                        LogType.NT),
                new BooleanLogItem("Is Continuous Input Enabled", () -> obj.isContinuousInputEnabled(),
                        LogType.DATALOG),
                new DoubleLogItem("Position Error", () -> obj.getPositionError(),
                        LogType.NT),
                new DoubleLogItem("Velocity Error", () -> obj.getVelocityError(),
                        LogType.DATALOG),
                new TunableDouble("Tunables/kP", obj.getP(), (d) -> obj.setP(d)),
                new TunableDouble("Tunables/kI", obj.getI(), (d) -> obj.setI(d)),
                new TunableDouble("Tunables/kD", obj.getD(), (d) -> obj.setD(d)),
                new TunableDouble("Tunables/Position Tolerance", obj.getPositionTolerance(),
                        (d) -> obj.setTolerance(d)),
                new TunableDouble("Tunables/Velocity Tolerance", obj.getVelocityTolerance(),
                        (d) -> obj.setTolerance(obj.getPositionTolerance(), d)),
        };
    }

    private static TrapezoidProfile.Constraints getConstraints(ProfiledPIDController obj) {
        try {
            Field f = ProfiledPIDController.class.getDeclaredField("m_constraints");
            f.setAccessible(true);
            return (TrapezoidProfile.Constraints) f.get(obj);
        } catch (Exception e) {
            e.printStackTrace();
            return new TrapezoidProfile.Constraints(0, 0);
        }

    }

    /**
     * Builds ProfiledPIDController log items.
     * 
     * @param obj the {@link PIDController} to use
     * @return the array of LogItems
     */
    public static AbstractLogItem<?>[] buildProfiledPIDControllerLogItems(ProfiledPIDController obj) {
        return new AbstractLogItem<?>[] {
                new DoubleLogItem("Setpoint Position", () -> obj.getSetpoint().position,
                        LogType.NT),
                new DoubleLogItem("Setpoint Velocity", () -> obj.getSetpoint().velocity,
                        LogType.NT),
                new BooleanLogItem("At Setpoint", () -> obj.atSetpoint(),
                        LogType.NT),
                new DoubleLogItem("Goal Position", () -> obj.getGoal().position,
                        LogType.NT),
                new DoubleLogItem("Goal Velocity", () -> obj.getGoal().velocity,
                        LogType.NT),
                new BooleanLogItem("At Goal", () -> obj.atGoal(),
                        LogType.NT),
                new DoubleLogItem("Position Error", () -> obj.getPositionError(),
                        LogType.NT),
                new DoubleLogItem("Velocity Error", () -> obj.getVelocityError(),
                        LogType.DATALOG),
                new TunableDouble("Tunables/kP", obj.getP(), (d) -> obj.setP(d)),
                new TunableDouble("Tunables/kI", obj.getI(), (d) -> obj.setI(d)),
                new TunableDouble("Tunables/kD", obj.getD(), (d) -> obj.setD(d)),
                new TunableDouble("Tunables/Position Tolerance", obj.getPositionTolerance(),
                        (d) -> obj.setTolerance(d)),
                new TunableDouble("Tunables/Velocity Tolerance", obj.getVelocityTolerance(),
                        (d) -> obj.setTolerance(obj.getPositionTolerance(), d)),
                new TunableDouble("Tunables/Velocity Constraint",
                        getConstraints(obj).maxVelocity,
                        (d) -> obj.setConstraints(
                                new TrapezoidProfile.Constraints(d, getConstraints(obj).maxAcceleration))),
                new TunableDouble("Tunables/Acceleration Constraint",
                        getConstraints(obj).maxAcceleration,
                        (d) -> obj.setConstraints(
                                new TrapezoidProfile.Constraints(getConstraints(obj).maxVelocity, d))),
        };
    }
}
