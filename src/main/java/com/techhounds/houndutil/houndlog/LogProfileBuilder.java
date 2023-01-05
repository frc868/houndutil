package com.techhounds.houndutil.houndlog;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import com.techhounds.houndutil.houndlog.enums.LogLevel;
import com.techhounds.houndutil.houndlog.logitems.BooleanLogItem;
import com.techhounds.houndutil.houndlog.logitems.DoubleLogItem;
import com.techhounds.houndutil.houndlog.logitems.FloatLogItem;
import com.techhounds.houndutil.houndlog.logitems.IntegerLogItem;
import com.techhounds.houndutil.houndlog.logitems.LogItem;
import com.techhounds.houndutil.houndlog.logitems.StringLogItem;

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
    public static LogItem<?>[] buildCANSparkMaxLogItems(CANSparkMax obj) {
        return new LogItem<?>[] {
                new DoubleLogItem("Encoder Distance", obj.getEncoder()::getPosition, LogLevel.MAIN),
                new DoubleLogItem("Encoder Distance Conversion Factor",
                        obj.getEncoder()::getPositionConversionFactor, LogLevel.DEBUG),
                new DoubleLogItem("Encoder Speed", obj.getEncoder()::getVelocity, LogLevel.MAIN),
                new DoubleLogItem("Encoder Speed Conversion Factor",
                        obj.getEncoder()::getVelocityConversionFactor, LogLevel.DEBUG),
                new DoubleLogItem("Closed Loop Ramp Rate", obj::getClosedLoopRampRate,
                        LogLevel.DEBUG),
                new DoubleLogItem("Open Loop Ramp Rate", obj::getOpenLoopRampRate, LogLevel.DEBUG),
                new DoubleLogItem("Speed", obj::get, LogLevel.MAIN),
                new DoubleLogItem("Bus Voltage", obj::getBusVoltage, LogLevel.MAIN),
                new DoubleLogItem("Motor Temperature", obj::getMotorTemperature, LogLevel.MAIN),
                new DoubleLogItem("Output Current", obj::getOutputCurrent, LogLevel.MAIN),
                new IntegerLogItem("Device ID", obj::getDeviceId, LogLevel.DEBUG),
                new StringLogItem("Firmware Version", obj::getFirmwareString, LogLevel.DEBUG),
                new BooleanLogItem("Is In Brake Mode",
                        () -> obj.getIdleMode() == CANSparkMax.IdleMode.kBrake, LogLevel.DEBUG),
                new BooleanLogItem("Is Inverted", obj::getInverted, LogLevel.DEBUG),
                new BooleanLogItem("Is Follower", obj::isFollower, LogLevel.DEBUG),
                new BooleanLogItem("Faults/Brownout",
                        () -> obj.getFault(CANSparkMax.FaultID.kBrownout), LogLevel.MAIN),
                new BooleanLogItem("Faults/Has Reset",
                        () -> obj.getFault(CANSparkMax.FaultID.kHasReset), LogLevel.INFO),
                new BooleanLogItem("Faults/Motor Fault",
                        () -> obj.getFault(CANSparkMax.FaultID.kMotorFault), LogLevel.INFO),
                new BooleanLogItem("Faults/Other Fault",
                        () -> obj.getFault(CANSparkMax.FaultID.kOtherFault), LogLevel.INFO),
                new BooleanLogItem("Faults/Overcurrent",
                        () -> obj.getFault(CANSparkMax.FaultID.kOvercurrent), LogLevel.INFO),
                new BooleanLogItem("Faults/Sensor Fault",
                        () -> obj.getFault(CANSparkMax.FaultID.kSensorFault), LogLevel.INFO),
                new BooleanLogItem("Faults/Stalled", () -> obj.getFault(CANSparkMax.FaultID.kStall),
                        LogLevel.INFO),
                new BooleanLogItem("Faults/DRV Fault",
                        () -> obj.getFault(CANSparkMax.FaultID.kStall), LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/Brownout",
                        () -> obj.getStickyFault(CANSparkMax.FaultID.kBrownout), LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/Has Reset",
                        () -> obj.getStickyFault(CANSparkMax.FaultID.kHasReset), LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/Motor Fault",
                        () -> obj.getStickyFault(CANSparkMax.FaultID.kMotorFault),
                        LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/Other Fault",
                        () -> obj.getStickyFault(CANSparkMax.FaultID.kOtherFault),
                        LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/Overcurrent",
                        () -> obj.getStickyFault(CANSparkMax.FaultID.kOvercurrent),
                        LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/Sensor Fault",
                        () -> obj.getStickyFault(CANSparkMax.FaultID.kSensorFault),
                        LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/Stalled",
                        () -> obj.getStickyFault(CANSparkMax.FaultID.kStall), LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/DRV Fault",
                        () -> obj.getStickyFault(CANSparkMax.FaultID.kStall), LogLevel.INFO),
        };
    }

    /**
     * Builds CANSparkMax log items.
     * 
     * @param obj the CANSparkMax object to use
     * @return the array of LogItems
     */
    public static LogItem<?>[] buildCANCoderLogItems(CANCoder obj) {
        return new LogItem<?>[] {
                new DoubleLogItem("Absolute Position", obj::getAbsolutePosition, LogLevel.MAIN),
                new DoubleLogItem("Position", obj::getPosition, LogLevel.MAIN),
                new DoubleLogItem("Velocity", obj::getVelocity, LogLevel.MAIN),
                new DoubleLogItem("Bus Voltage", obj::getBusVoltage, LogLevel.MAIN),
                new IntegerLogItem("Device ID", obj::getDeviceID, LogLevel.DEBUG),
                new StringLogItem("Magnetic Field Strength", obj.getMagnetFieldStrength()::name,
                        LogLevel.INFO),
        };
    }

    /**
     * Builds navX log items.
     * 
     * @param obj the navx to use
     * @return the array of LogItems
     */
    public static LogItem<?>[] buildNavXLogItems(AHRS obj) {
        return new LogItem<?>[] {
                new FloatLogItem("Pitch", obj::getPitch, LogLevel.MAIN),
                new FloatLogItem("Roll", obj::getRoll, LogLevel.MAIN),
                new FloatLogItem("Yaw", obj::getYaw, LogLevel.MAIN),
                new DoubleLogItem("Angle Rotation Rate", obj::getRate, LogLevel.MAIN),
                new FloatLogItem("X Axis Acceleration", obj::getWorldLinearAccelX, LogLevel.MAIN),
                new FloatLogItem("Y Axis Acceleration", obj::getWorldLinearAccelY, LogLevel.MAIN),
                new FloatLogItem("Z Axis Acceleration", obj::getWorldLinearAccelZ, LogLevel.MAIN),
                new FloatLogItem("Compass Heading", obj::getCompassHeading, LogLevel.MAIN),
                new BooleanLogItem("Is Calibrating", obj::isCalibrating, LogLevel.MAIN),
                new BooleanLogItem("Is Magnetometer Calibrated", obj::isMagnetometerCalibrated,
                        LogLevel.MAIN),
                new BooleanLogItem("Is Connected", obj::isConnected, LogLevel.MAIN),
                new BooleanLogItem("Is Moving", obj::isMoving, LogLevel.MAIN),
                new BooleanLogItem("Is Rotating", obj::isRotating, LogLevel.MAIN),
                new BooleanLogItem("Is Magnetic Disturbance", obj::isMagneticDisturbance,
                        LogLevel.MAIN),
                new FloatLogItem("Temperature", obj::getTempC, LogLevel.MAIN),
                new DoubleLogItem("Update Count", obj::getUpdateCount, LogLevel.DEBUG),
                new StringLogItem("Firmware Version", obj::getFirmwareVersion, LogLevel.DEBUG)

        };
    }

    /**
     * Builds Pigeon 2 log items.
     * 
     * @param obj the navx to use
     * @return the array of LogItems
     */
    public static LogItem<?>[] buildPigeon2LogItems(Pigeon2 obj) {

        return new LogItem<?>[] {
                new DoubleLogItem("Pitch", obj::getPitch, LogLevel.MAIN),
                new DoubleLogItem("Roll", obj::getRoll, LogLevel.MAIN),
                new DoubleLogItem("Yaw", obj::getYaw, LogLevel.MAIN),
                new DoubleLogItem("YawRad", () -> Units.degreesToRadians(obj.getYaw()), LogLevel.MAIN),
                new DoubleLogItem("Absolute Compass Heading", obj::getAbsoluteCompassHeading,
                        LogLevel.INFO),
                new IntegerLogItem("Uptime", obj::getUpTime, LogLevel.INFO),
                new DoubleLogItem("Temperature", obj::getTemp, LogLevel.INFO),
                new StringLogItem("Last Error", obj.getLastError()::toString, LogLevel.DEBUG),
                new IntegerLogItem("Device ID", obj::getDeviceID, LogLevel.DEBUG),
                new IntegerLogItem("Firmware Version", obj::getFirmwareVersion, LogLevel.DEBUG),
        };
    }

    /**
     * Builds DoubleSolenoid log items.
     * 
     * @param obj the {@link DoubleSolenoid} to use
     * @return the array of LogItems
     */
    public static LogItem<?>[] buildDoubleSolenoidLogItems(DoubleSolenoid obj) {
        return new LogItem<?>[] {
                new BooleanLogItem("Position", () -> obj.get() == DoubleSolenoid.Value.kForward,
                        LogLevel.MAIN),
                new DoubleLogItem("Forward Channel", () -> (double) obj.getFwdChannel(),
                        LogLevel.DEBUG),
                new DoubleLogItem("Reverse Channel", () -> (double) obj.getRevChannel(),
                        LogLevel.DEBUG),
                new BooleanLogItem("Is Forward Solenoid Disabled", obj::isFwdSolenoidDisabled,
                        LogLevel.INFO),
                new BooleanLogItem("Is Reverse Solenoid Disabled", obj::isRevSolenoidDisabled,
                        LogLevel.INFO)
        };
    }

    /**
     * Builds PDH log items.
     * 
     * @param obj the PDH instance to use
     * @return the array of LogItems
     */
    public static LogItem<?>[] buildPDHLogItems(PowerDistribution obj) {
        return new LogItem<?>[] {
                new DoubleLogItem("Voltage", obj::getVoltage, LogLevel.MAIN),
                new DoubleLogItem("Temperature", obj::getTemperature, LogLevel.MAIN),
                new DoubleLogItem("Total Current (A)", obj::getTotalCurrent, LogLevel.MAIN),
                new DoubleLogItem("Total Power (W)", obj::getTotalPower, LogLevel.MAIN),
                new DoubleLogItem("Total Energy (J)", obj::getTotalEnergy, LogLevel.MAIN),
                new DoubleLogItem("Channel 0 Current", () -> obj.getCurrent(0), LogLevel.INFO),
                new DoubleLogItem("Channel 1 Current", () -> obj.getCurrent(1), LogLevel.INFO),
                new DoubleLogItem("Channel 2 Current", () -> obj.getCurrent(2), LogLevel.INFO),
                new DoubleLogItem("Channel 3 Current", () -> obj.getCurrent(3), LogLevel.INFO),
                new DoubleLogItem("Channel 4 Current", () -> obj.getCurrent(4), LogLevel.INFO),
                new DoubleLogItem("Channel 5 Current", () -> obj.getCurrent(5), LogLevel.INFO),
                new DoubleLogItem("Channel 6 Current", () -> obj.getCurrent(6), LogLevel.INFO),
                new DoubleLogItem("Channel 7 Current", () -> obj.getCurrent(7), LogLevel.INFO),
                new DoubleLogItem("Channel 8 Current", () -> obj.getCurrent(8), LogLevel.INFO),
                new DoubleLogItem("Channel 9 Current", () -> obj.getCurrent(9), LogLevel.INFO),
                new DoubleLogItem("Channel 10 Current", () -> obj.getCurrent(10), LogLevel.INFO),
                new DoubleLogItem("Channel 11 Current", () -> obj.getCurrent(11), LogLevel.INFO),
                new DoubleLogItem("Channel 12 Current", () -> obj.getCurrent(12), LogLevel.INFO),
                new DoubleLogItem("Channel 13 Current", () -> obj.getCurrent(13), LogLevel.INFO),
                new DoubleLogItem("Channel 14 Current", () -> obj.getCurrent(14), LogLevel.INFO),
                new DoubleLogItem("Channel 15 Current", () -> obj.getCurrent(15), LogLevel.INFO),
                new DoubleLogItem("Channel 16 Current", () -> obj.getCurrent(16), LogLevel.INFO),
                new DoubleLogItem("Channel 17 Current", () -> obj.getCurrent(17), LogLevel.INFO),
                new DoubleLogItem("Channel 18 Current", () -> obj.getCurrent(18), LogLevel.INFO),
                new DoubleLogItem("Channel 19 Current", () -> obj.getCurrent(19), LogLevel.INFO),
                new DoubleLogItem("Channel 20 Current", () -> obj.getCurrent(20), LogLevel.INFO),
                new DoubleLogItem("Channel 21 Current", () -> obj.getCurrent(21), LogLevel.INFO),
                new DoubleLogItem("Channel 22 Current", () -> obj.getCurrent(22), LogLevel.INFO),
                new DoubleLogItem("Channel 23 Current", () -> obj.getCurrent(23), LogLevel.INFO),
                new DoubleLogItem("Version/Firmware Major",
                        () -> (double) obj.getVersion().firmwareMajor, LogLevel.DEBUG),
                new DoubleLogItem("Version/Firmware Minor",
                        () -> (double) obj.getVersion().firmwareMinor, LogLevel.DEBUG),
                new DoubleLogItem("Version/Firmware Fix",
                        () -> (double) obj.getVersion().firmwareFix, LogLevel.DEBUG),
                new DoubleLogItem("Version/Hardware Major",
                        () -> (double) obj.getVersion().hardwareMajor, LogLevel.DEBUG),
                new DoubleLogItem("Version/Hardware Minor",
                        () -> (double) obj.getVersion().hardwareMinor, LogLevel.DEBUG),
                new DoubleLogItem("Unique ID", () -> (double) obj.getVersion().uniqueId,
                        LogLevel.DEBUG),
                new StringLogItem("Type",
                        () -> obj.getType() == ModuleType.kRev ? "Rev PDH" : "CTRE PDP",
                        LogLevel.INFO),
                new BooleanLogItem("Is Switchable Channel On", obj::getSwitchableChannel,
                        LogLevel.DEBUG),
                new BooleanLogItem("Faults/Channel 0 Breaker Fault",
                        () -> obj.getFaults().Channel0BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Faults/Channel 1 Breaker Fault",
                        () -> obj.getFaults().Channel1BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Faults/Channel 2 Breaker Fault",
                        () -> obj.getFaults().Channel2BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Faults/Channel 3 Breaker Fault",
                        () -> obj.getFaults().Channel3BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Faults/Channel 4 Breaker Fault",
                        () -> obj.getFaults().Channel4BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Faults/Channel 5 Breaker Fault",
                        () -> obj.getFaults().Channel5BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Faults/Channel 6 Breaker Fault",
                        () -> obj.getFaults().Channel6BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Faults/Channel 7 Breaker Fault",
                        () -> obj.getFaults().Channel7BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Faults/Channel 8 Breaker Fault",
                        () -> obj.getFaults().Channel8BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Faults/Channel 9 Breaker Fault",
                        () -> obj.getFaults().Channel9BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Faults/Channel 10 Breaker Fault",
                        () -> obj.getFaults().Channel11BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Faults/Channel 11 Breaker Fault",
                        () -> obj.getFaults().Channel10BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Faults/Channel 12 Breaker Fault",
                        () -> obj.getFaults().Channel12BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Faults/Channel 13 Breaker Fault",
                        () -> obj.getFaults().Channel13BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Faults/Channel 14 Breaker Fault",
                        () -> obj.getFaults().Channel14BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Faults/Channel 15 Breaker Fault",
                        () -> obj.getFaults().Channel15BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Faults/Channel 16 Breaker Fault",
                        () -> obj.getFaults().Channel16BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Faults/Channel 17 Breaker Fault",
                        () -> obj.getFaults().Channel17BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Faults/Channel 18 Breaker Fault",
                        () -> obj.getFaults().Channel18BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Faults/Channel 19 Breaker Fault",
                        () -> obj.getFaults().Channel19BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Faults/Channel 20 Breaker Fault",
                        () -> obj.getFaults().Channel20BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Faults/Channel 21 Breaker Fault",
                        () -> obj.getFaults().Channel21BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Faults/Channel 22 Breaker Fault",
                        () -> obj.getFaults().Channel22BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Faults/Channel 23 Breaker Fault",
                        () -> obj.getFaults().Channel23BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Faults/Brownout", () -> obj.getFaults().Brownout, LogLevel.INFO),
                new BooleanLogItem("Faults/CAN Warning",
                        () -> obj.getFaults().CanWarning, LogLevel.INFO),
                new BooleanLogItem("Faults/Hardware Fault", () -> obj.getFaults().HardwareFault,
                        LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/Channel 0 Breaker Fault",
                        () -> obj.getStickyFaults().Channel0BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/Channel 1 Breaker Fault",
                        () -> obj.getStickyFaults().Channel1BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/Channel 2 Breaker Fault",
                        () -> obj.getStickyFaults().Channel2BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/Channel 3 Breaker Fault",
                        () -> obj.getStickyFaults().Channel3BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/Channel 4 Breaker Fault",
                        () -> obj.getStickyFaults().Channel4BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/Channel 5 Breaker Fault",
                        () -> obj.getStickyFaults().Channel5BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/Channel 6 Breaker Fault",
                        () -> obj.getStickyFaults().Channel6BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/Channel 7 Breaker Fault",
                        () -> obj.getStickyFaults().Channel7BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/Channel 8 Breaker Fault",
                        () -> obj.getStickyFaults().Channel8BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/Channel 9 Breaker Fault",
                        () -> obj.getStickyFaults().Channel9BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/Channel 10 Breaker Fault",
                        () -> obj.getStickyFaults().Channel11BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/Channel 11 Breaker Fault",
                        () -> obj.getStickyFaults().Channel10BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/Channel 12 Breaker Fault",
                        () -> obj.getStickyFaults().Channel12BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/Channel 13 Breaker Fault",
                        () -> obj.getStickyFaults().Channel13BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/Channel 14 Breaker Fault",
                        () -> obj.getStickyFaults().Channel14BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/Channel 15 Breaker Fault",
                        () -> obj.getStickyFaults().Channel15BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/Channel 16 Breaker Fault",
                        () -> obj.getStickyFaults().Channel16BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/Channel 17 Breaker Fault",
                        () -> obj.getStickyFaults().Channel17BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/Channel 18 Breaker Fault",
                        () -> obj.getStickyFaults().Channel18BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/Channel 19 Breaker Fault",
                        () -> obj.getStickyFaults().Channel19BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/Channel 20 Breaker Fault",
                        () -> obj.getStickyFaults().Channel20BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/Channel 21 Breaker Fault",
                        () -> obj.getStickyFaults().Channel21BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/Channel 22 Breaker Fault",
                        () -> obj.getStickyFaults().Channel22BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/Channel 23 Breaker Fault",
                        () -> obj.getStickyFaults().Channel23BreakerFault, LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/Brownout", () -> obj.getStickyFaults().Brownout,
                        LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/CAN Bus Off",
                        () -> obj.getStickyFaults().CanBusOff, LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/CAN Warning",
                        () -> obj.getStickyFaults().CanWarning, LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/Has Reset", () -> obj.getStickyFaults().HasReset,
                        LogLevel.INFO)

        };
    }

    /**
     * Builds REV Pneumatics Hub log items.
     * 
     * @param obj the PH instance to use
     * @return the array of LogItems
     */
    public static LogItem<?>[] buildPneumaticHubLogItems(PneumaticHub obj) {
        return new LogItem<?>[] {
                new DoubleLogItem("Input Voltage", obj::getInputVoltage, LogLevel.MAIN),
                new DoubleLogItem("5V Regulated Voltage", obj::get5VRegulatedVoltage, LogLevel.MAIN),
                new DoubleLogItem("Module Number", () -> (double) obj.getModuleNumber(),
                        LogLevel.INFO),
                new DoubleLogItem("Channel 0 Pressure", () -> obj.getPressure(0), LogLevel.INFO),
                new DoubleLogItem("Channel 1 Pressure", () -> obj.getPressure(1), LogLevel.INFO),
                new DoubleLogItem("Channel 2 Pressure", () -> obj.getPressure(2), LogLevel.INFO),
                new DoubleLogItem("Channel 3 Pressure", () -> obj.getPressure(3), LogLevel.INFO),
                new DoubleLogItem("Channel 4 Pressure", () -> obj.getPressure(4), LogLevel.INFO),
                new DoubleLogItem("Channel 5 Pressure", () -> obj.getPressure(5), LogLevel.INFO),
                new DoubleLogItem("Channel 6 Pressure", () -> obj.getPressure(6), LogLevel.INFO),
                new DoubleLogItem("Channel 7 Pressure", () -> obj.getPressure(7), LogLevel.INFO),
                new DoubleLogItem("Channel 8 Pressure", () -> obj.getPressure(8), LogLevel.INFO),
                new DoubleLogItem("Channel 9 Pressure", () -> obj.getPressure(9), LogLevel.INFO),
                new DoubleLogItem("Channel 10 Pressure", () -> obj.getPressure(10), LogLevel.INFO),
                new DoubleLogItem("Channel 11 Pressure", () -> obj.getPressure(11), LogLevel.INFO),
                new DoubleLogItem("Channel 12 Pressure", () -> obj.getPressure(12), LogLevel.INFO),
                new DoubleLogItem("Channel 13 Pressure", () -> obj.getPressure(13), LogLevel.INFO),
                new DoubleLogItem("Channel 14 Pressure", () -> obj.getPressure(14), LogLevel.INFO),
                new DoubleLogItem("Channel 15 Pressure", () -> obj.getPressure(15), LogLevel.INFO),
                new BooleanLogItem("Pressure Switch", obj::getPressureSwitch, LogLevel.INFO),
                new DoubleLogItem("Compressor Current", obj::getCompressorCurrent, LogLevel.MAIN),
                new BooleanLogItem("Is Compressor Running", obj::getCompressor, LogLevel.MAIN),
                new DoubleLogItem("Solenoids", () -> (double) obj.getSolenoids(), LogLevel.INFO),
                new DoubleLogItem("Solenoids Total Current", obj::getSolenoidsTotalCurrent,
                        LogLevel.MAIN),
                new DoubleLogItem("Solenoids Voltage", obj::getSolenoidsVoltage, LogLevel.MAIN),
                new DoubleLogItem("Version/Firmware Major",
                        () -> (double) obj.getVersion().firmwareMajor, LogLevel.DEBUG),
                new DoubleLogItem("Version/Firmware Minor",
                        () -> (double) obj.getVersion().firmwareMinor, LogLevel.DEBUG),
                new DoubleLogItem("Version/Firmware Fix",
                        () -> (double) obj.getVersion().firmwareFix, LogLevel.DEBUG),
                new DoubleLogItem("Version/Hardware Major",
                        () -> (double) obj.getVersion().hardwareMajor, LogLevel.DEBUG),
                new DoubleLogItem("Version/Hardware Minor",
                        () -> (double) obj.getVersion().hardwareMinor, LogLevel.DEBUG),
                new BooleanLogItem("Faults/Brownout", () -> obj.getFaults().Brownout, LogLevel.MAIN),
                new BooleanLogItem("Faults/CAN Warning", () -> obj.getFaults().CanWarning,
                        LogLevel.INFO),
                new BooleanLogItem("Faults/Channel 0 Fault", () -> obj.getFaults().Channel0Fault,
                        LogLevel.INFO),
                new BooleanLogItem("Faults/Channel 1 Fault", () -> obj.getFaults().Channel1Fault,
                        LogLevel.INFO),
                new BooleanLogItem("Faults/Channel 2 Fault", () -> obj.getFaults().Channel2Fault,
                        LogLevel.INFO),
                new BooleanLogItem("Faults/Channel 3 Fault", () -> obj.getFaults().Channel3Fault,
                        LogLevel.INFO),
                new BooleanLogItem("Faults/Channel 4 Fault", () -> obj.getFaults().Channel4Fault,
                        LogLevel.INFO),
                new BooleanLogItem("Faults/Channel 5 Fault", () -> obj.getFaults().Channel5Fault,
                        LogLevel.INFO),
                new BooleanLogItem("Faults/Channel 6 Fault", () -> obj.getFaults().Channel6Fault,
                        LogLevel.INFO),
                new BooleanLogItem("Faults/Channel 7 Fault", () -> obj.getFaults().Channel7Fault,
                        LogLevel.INFO),
                new BooleanLogItem("Faults/Channel 8 Fault", () -> obj.getFaults().Channel8Fault,
                        LogLevel.INFO),
                new BooleanLogItem("Faults/Channel 9 Fault", () -> obj.getFaults().Channel9Fault,
                        LogLevel.INFO),
                new BooleanLogItem("Faults/Channel 10 Fault", () -> obj.getFaults().Channel10Fault,
                        LogLevel.INFO),
                new BooleanLogItem("Faults/Channel 11 Fault", () -> obj.getFaults().Channel11Fault,
                        LogLevel.INFO),
                new BooleanLogItem("Faults/Channel 12 Fault", () -> obj.getFaults().Channel12Fault,
                        LogLevel.INFO),
                new BooleanLogItem("Faults/Channel 13 Fault", () -> obj.getFaults().Channel13Fault,
                        LogLevel.INFO),
                new BooleanLogItem("Faults/Channel 14 Fault", () -> obj.getFaults().Channel14Fault,
                        LogLevel.INFO),
                new BooleanLogItem("Faults/Channel 15 Fault", () -> obj.getFaults().Channel15Fault,
                        LogLevel.INFO),
                new BooleanLogItem("Faults/Compressor Open", () -> obj.getFaults().CompressorOpen,
                        LogLevel.MAIN),
                new BooleanLogItem("Faults/Compressor Over Current",
                        () -> obj.getFaults().CompressorOverCurrent, LogLevel.MAIN),
                new BooleanLogItem("Faults/Hardware Fault", () -> obj.getFaults().HardwareFault,
                        LogLevel.MAIN),
                new BooleanLogItem("Faults/Solenoid Over Current",
                        () -> obj.getFaults().SolenoidOverCurrent, LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/Brownout", () -> obj.getStickyFaults().Brownout,
                        LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/CAN Bus Off",
                        () -> obj.getStickyFaults().CanBusOff, LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/CAN Warning",
                        () -> obj.getStickyFaults().CanWarning, LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/Compressor Open",
                        () -> obj.getStickyFaults().CompressorOpen, LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/Compressor Over Current",
                        () -> obj.getStickyFaults().CompressorOverCurrent, LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/Has Reset", () -> obj.getStickyFaults().HasReset,
                        LogLevel.INFO),
                new BooleanLogItem("Sticky Faults/Solenoid Over Current",
                        () -> obj.getStickyFaults().SolenoidOverCurrent, LogLevel.INFO)

        };
    }

}
