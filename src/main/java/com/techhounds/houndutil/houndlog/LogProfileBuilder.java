package com.techhounds.houndutil.houndlog;

import java.lang.reflect.Field;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import com.techhounds.houndutil.houndlog.enums.LogType;
import com.techhounds.houndutil.houndlog.logitems.AbstractLogItem;
import com.techhounds.houndutil.houndlog.logitems.BooleanArrayLogItem;
import com.techhounds.houndutil.houndlog.logitems.BooleanLogItem;
import com.techhounds.houndutil.houndlog.logitems.DoubleArrayLogItem;
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
    public static AbstractLogItem<?>[] buildTalonFXLogItems(TalonFX obj) {

        return new AbstractLogItem<?>[] {
                new DoubleLogItem("position", () -> obj.getPosition().getValue(), LogType.NT),
                new DoubleLogItem("velocity", () -> obj.getVelocity().getValue(), LogType.NT),
                new DoubleLogItem("acceleration", () -> obj.getAcceleration().getValue(), LogType.NT),
                new DoubleLogItem("temperature", () -> obj.getDeviceTemp().getValue(), LogType.NT),
                new DoubleLogItem("speed", () -> obj.getDutyCycle().getValue() / 2.0, LogType.NT), // [-2, 2)
                new DoubleLogItem("outputVoltage", () -> obj.getMotorVoltage().getValue(), LogType.NT),
                new DoubleLogItem("busVoltage", () -> obj.getSupplyVoltage().getValue(), LogType.NT),
                new DoubleLogItem("outputCurrent", () -> obj.getTorqueCurrent().getValue(), LogType.NT),
                new StringLogItem("bridgeOutput", () -> obj.getBridgeOutput().getValue().toString(), LogType.NT),
                new DoubleLogItem("closedLoopReference", () -> obj.getClosedLoopReference().getValue(), LogType.NT),
                new DoubleLogItem("closedLoopOutput", () -> obj.getClosedLoopOutput().getValue(), LogType.NT),
                new DoubleLogItem("closedLoopError", () -> obj.getClosedLoopError().getValue(), LogType.NT),
                // new BooleanLogItem("faults/bootDuringEnable", () ->
                // obj.getFault_BootDuringEnable().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/bridgeBrownout", () ->
                // obj.getFault_BridgeBrownout().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/deviceTemp", () ->
                // obj.getFault_DeviceTemp().getValue(), LogType.DATALOG),
                // new BooleanLogItem("faults/forwardHardLimit", () ->
                // obj.getFault_ForwardHardLimit().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/forwardSoftLimit", () ->
                // obj.getFault_ForwardSoftLimit().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/fusedSensorOutOfSync", () ->
                // obj.getFault_FusedSensorOutOfSync().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/hardware", () ->
                // obj.getFault_Hardware().getValue(), LogType.DATALOG),
                // new BooleanLogItem("faults/missingDifferentialFX",
                // () -> obj.getFault_MissingDifferentialFX().getValue(), LogType.DATALOG),
                // new BooleanLogItem("faults/overSupplyV", () ->
                // obj.getFault_OverSupplyV().getValue(), LogType.DATALOG),
                // new BooleanLogItem("faults/procTemp", () ->
                // obj.getFault_ProcTemp().getValue(), LogType.DATALOG),
                // new BooleanLogItem("faults/remoteSensorDataInvalid",
                // () -> obj.getFault_RemoteSensorDataInvalid().getValue(), LogType.DATALOG),
                // new BooleanLogItem("faults/remoteSensorPosOverflow",
                // () -> obj.getFault_RemoteSensorPosOverflow().getValue(), LogType.DATALOG),
                // new BooleanLogItem("faults/remoteSensorReset", () ->
                // obj.getFault_RemoteSensorReset().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/reverseHardLimit", () ->
                // obj.getFault_ReverseHardLimit().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/reverseSoftLimit", () ->
                // obj.getFault_ReverseSoftLimit().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/statorCurrLimit", () ->
                // obj.getFault_StatorCurrLimit().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/supplyCurrLimit", () ->
                // obj.getFault_SupplyCurrLimit().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/undervoltage", () ->
                // obj.getFault_Undervoltage().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/unlicensedFeatureInUse",
                // () -> obj.getFault_UnlicensedFeatureInUse().getValue(), LogType.DATALOG),
                // new BooleanLogItem("faults/unstableSupplyV", () ->
                // obj.getFault_UnstableSupplyV().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/usingFusedCANcoderWhileUnlicensed",
                // () -> obj.getStickyFault_UsingFusedCANcoderWhileUnlicensed().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/sticky/bootDuringEnable",
                // () -> obj.getStickyFault_BootDuringEnable().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/sticky/bridgeBrownout", () ->
                // obj.getStickyFault_BridgeBrownout().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/sticky/deviceTemp", () ->
                // obj.getStickyFault_DeviceTemp().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/sticky/forwardHardLimit",
                // () -> obj.getStickyFault_ForwardHardLimit().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/sticky/forwardSoftLimit",
                // () -> obj.getStickyFault_ForwardSoftLimit().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/sticky/fusedSensorOutOfSync",
                // () -> obj.getStickyFault_FusedSensorOutOfSync().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/sticky/hardware", () ->
                // obj.getStickyFault_Hardware().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/sticky/missingDifferentialFX",
                // () -> obj.getStickyFault_MissingDifferentialFX().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/sticky/overSupplyV", () ->
                // obj.getStickyFault_OverSupplyV().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/sticky/procTemp", () ->
                // obj.getStickyFault_ProcTemp().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/sticky/remoteSensorDataInvalid",
                // () -> obj.getStickyFault_RemoteSensorDataInvalid().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/sticky/remoteSensorPosOverflow",
                // () -> obj.getStickyFault_RemoteSensorPosOverflow().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/sticky/remoteSensorReset",
                // () -> obj.getStickyFault_RemoteSensorReset().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/sticky/reverseHardLimit",
                // () -> obj.getStickyFault_ReverseHardLimit().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/sticky/reverseSoftLimit",
                // () -> obj.getStickyFault_ReverseSoftLimit().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/sticky/statorCurrLimit",
                // () -> obj.getStickyFault_StatorCurrLimit().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/sticky/supplyCurrLimit",
                // () -> obj.getStickyFault_SupplyCurrLimit().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/sticky/undervoltage", () ->
                // obj.getStickyFault_Undervoltage().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/sticky/unlicensedFeatureInUse",
                // () -> obj.getStickyFault_UnlicensedFeatureInUse().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/sticky/unstableSupplyV",
                // () -> obj.getStickyFault_UnstableSupplyV().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/sticky/usingFusedCANcoderWhileUnlicensed",
                // () -> obj.getStickyFault_UsingFusedCANcoderWhileUnlicensed().getValue(),
                // LogType.DATALOG)

        };
    }

    public static AbstractLogItem<?>[] buildCANSparkMaxLogItems(CANSparkMax obj) {
        return new AbstractLogItem<?>[] {
                new DoubleLogItem("encoderPosition", obj.getEncoder()::getPosition, LogType.NT),
                new DoubleLogItem("encoderPositionConversionFactor",
                        obj.getEncoder()::getPositionConversionFactor, LogType.DATALOG),
                new DoubleLogItem("encoderVelocity", obj.getEncoder()::getVelocity, LogType.NT),
                new DoubleLogItem("encoderVelocityConversionFactor",
                        obj.getEncoder()::getVelocityConversionFactor, LogType.DATALOG),
                new DoubleLogItem("speed", obj::get, LogType.NT),
                new DoubleLogItem("outputVoltage", obj::getAppliedOutput, LogType.NT),
                new DoubleLogItem("busVoltage", obj::getBusVoltage, LogType.DATALOG),
                new DoubleLogItem("motorTemperature", obj::getMotorTemperature, LogType.NT),
                new DoubleLogItem("outputCurrent", obj::getOutputCurrent, LogType.NT),
                new IntegerLogItem("deviceId", obj::getDeviceId, LogType.DATALOG),
                new StringLogItem("firmwareVersion", obj::getFirmwareString, LogType.DATALOG),
                new BooleanLogItem("brakeMode", () -> obj.getIdleMode() == CANSparkBase.IdleMode.kBrake, LogType.NT),
                new BooleanLogItem("isInverted", obj::getInverted, LogType.DATALOG),
                new BooleanLogItem("isFollower", obj::isFollower, LogType.DATALOG),
                new BooleanLogItem("faults/brownout",
                        () -> obj.getFault(CANSparkMax.FaultID.kBrownout), LogType.NT),
                new BooleanLogItem("faults/hasReset",
                        () -> obj.getFault(CANSparkMax.FaultID.kHasReset), LogType.DATALOG),
                new BooleanLogItem("faults/motorFault",
                        () -> obj.getFault(CANSparkMax.FaultID.kMotorFault), LogType.DATALOG),
                new BooleanLogItem("faults/otherFault",
                        () -> obj.getFault(CANSparkMax.FaultID.kOtherFault), LogType.DATALOG),
                new BooleanLogItem("faults/overcurrent",
                        () -> obj.getFault(CANSparkMax.FaultID.kOvercurrent), LogType.DATALOG),
                new BooleanLogItem("faults/sensorFault",
                        () -> obj.getFault(CANSparkMax.FaultID.kSensorFault), LogType.DATALOG),
                new BooleanLogItem("faults/stalled",
                        () -> obj.getFault(CANSparkMax.FaultID.kStall), LogType.DATALOG),
                new BooleanLogItem("faults/drvFault",
                        () -> obj.getFault(CANSparkMax.FaultID.kDRVFault), LogType.DATALOG),
                new BooleanLogItem("faults/sticky/brownout",
                        () -> obj.getStickyFault(CANSparkMax.FaultID.kBrownout), LogType.DATALOG),
                new BooleanLogItem("faults/sticky/hasReset",
                        () -> obj.getStickyFault(CANSparkMax.FaultID.kHasReset), LogType.DATALOG),
                new BooleanLogItem("faults/sticky/motorFault",
                        () -> obj.getStickyFault(CANSparkMax.FaultID.kMotorFault), LogType.DATALOG),
                new BooleanLogItem("faults/sticky/otherFault",
                        () -> obj.getStickyFault(CANSparkMax.FaultID.kOtherFault), LogType.DATALOG),
                new BooleanLogItem("faults/sticky/overcurrent",
                        () -> obj.getStickyFault(CANSparkMax.FaultID.kOvercurrent), LogType.DATALOG),
                new BooleanLogItem("faults/sticky/sensorFault",
                        () -> obj.getStickyFault(CANSparkMax.FaultID.kSensorFault), LogType.DATALOG),
                new BooleanLogItem("faults/sticky/stalled",
                        () -> obj.getStickyFault(CANSparkMax.FaultID.kStall), LogType.DATALOG),
                new BooleanLogItem("faults/sticky/drvFault",
                        () -> obj.getStickyFault(CANSparkMax.FaultID.kDRVFault), LogType.DATALOG),
        };
    }

    public static AbstractLogItem<?>[] buildCANSparkFlexLogItems(CANSparkFlex obj) {
        return new AbstractLogItem<?>[] {
                new DoubleLogItem("encoderPosition", obj.getEncoder()::getPosition, LogType.NT),
                new DoubleLogItem("encoderPositionConversionFactor",
                        obj.getEncoder()::getPositionConversionFactor, LogType.DATALOG),
                new DoubleLogItem("encoderVelocity", obj.getEncoder()::getVelocity, LogType.NT),
                new DoubleLogItem("encoderVelocityConversionFactor",
                        obj.getEncoder()::getVelocityConversionFactor, LogType.DATALOG),
                new DoubleLogItem("speed", obj::get, LogType.NT),
                new DoubleLogItem("outputVoltage", obj::getAppliedOutput, LogType.NT),
                new DoubleLogItem("busVoltage", obj::getBusVoltage, LogType.DATALOG),
                new DoubleLogItem("motorTemperature", obj::getMotorTemperature, LogType.NT),
                new DoubleLogItem("outputCurrent", obj::getOutputCurrent, LogType.NT),
                new IntegerLogItem("deviceId", obj::getDeviceId, LogType.DATALOG),
                new StringLogItem("firmwareVersion", obj::getFirmwareString, LogType.DATALOG),
                new BooleanLogItem("brakeMode", () -> obj.getIdleMode() == CANSparkBase.IdleMode.kBrake, LogType.NT),
                new BooleanLogItem("isInverted", obj::getInverted, LogType.DATALOG),
                new BooleanLogItem("isFollower", obj::isFollower, LogType.DATALOG),
                new BooleanLogItem("faults/brownout",
                        () -> obj.getFault(CANSparkFlex.FaultID.kBrownout), LogType.NT),
                new BooleanLogItem("faults/hasReset",
                        () -> obj.getFault(CANSparkFlex.FaultID.kHasReset), LogType.DATALOG),
                new BooleanLogItem("faults/motorFault",
                        () -> obj.getFault(CANSparkFlex.FaultID.kMotorFault), LogType.DATALOG),
                new BooleanLogItem("faults/otherFault",
                        () -> obj.getFault(CANSparkFlex.FaultID.kOtherFault), LogType.DATALOG),
                new BooleanLogItem("faults/overcurrent",
                        () -> obj.getFault(CANSparkFlex.FaultID.kOvercurrent), LogType.DATALOG),
                new BooleanLogItem("faults/sensorFault",
                        () -> obj.getFault(CANSparkFlex.FaultID.kSensorFault), LogType.DATALOG),
                new BooleanLogItem("faults/stalled",
                        () -> obj.getFault(CANSparkFlex.FaultID.kStall), LogType.DATALOG),
                new BooleanLogItem("faults/drvFault",
                        () -> obj.getFault(CANSparkFlex.FaultID.kDRVFault), LogType.DATALOG),
                new BooleanLogItem("faults/sticky/brownout",
                        () -> obj.getStickyFault(CANSparkFlex.FaultID.kBrownout), LogType.DATALOG),
                new BooleanLogItem("faults/sticky/hasReset",
                        () -> obj.getStickyFault(CANSparkFlex.FaultID.kHasReset), LogType.DATALOG),
                new BooleanLogItem("faults/sticky/motorFault",
                        () -> obj.getStickyFault(CANSparkFlex.FaultID.kMotorFault), LogType.DATALOG),
                new BooleanLogItem("faults/sticky/otherFault",
                        () -> obj.getStickyFault(CANSparkFlex.FaultID.kOtherFault), LogType.DATALOG),
                new BooleanLogItem("faults/sticky/overcurrent",
                        () -> obj.getStickyFault(CANSparkFlex.FaultID.kOvercurrent), LogType.DATALOG),
                new BooleanLogItem("faults/sticky/sensorFault",
                        () -> obj.getStickyFault(CANSparkFlex.FaultID.kSensorFault), LogType.DATALOG),
                new BooleanLogItem("faults/sticky/stalled",
                        () -> obj.getStickyFault(CANSparkFlex.FaultID.kStall), LogType.DATALOG),
                new BooleanLogItem("faults/sticky/drvFault",
                        () -> obj.getStickyFault(CANSparkFlex.FaultID.kDRVFault), LogType.DATALOG),
        };
    }

    /**
     * Builds CANSparkMax log items.
     * 
     * @param obj the CANSparkMax object to use
     * @return the array of LogItems
     */
    public static AbstractLogItem<?>[] buildCANcoderLogItems(CANcoder obj) {
        return new AbstractLogItem<?>[] {
                new DoubleLogItem("absolutePosition", () -> obj.getAbsolutePosition().getValue(), LogType.NT),
                new DoubleLogItem("position", () -> obj.getPosition().getValue(), LogType.NT),
                new DoubleLogItem("velocity", () -> obj.getVelocity().getValue(), LogType.NT),
                new DoubleLogItem("busVoltage", () -> obj.getSupplyVoltage().getValue(), LogType.DATALOG),
                new IntegerLogItem("deviceId", obj::getDeviceID, LogType.DATALOG),
                new StringLogItem("magnetHealth", () -> obj.getMagnetHealth().getValue().name(), LogType.DATALOG),
                // new BooleanLogItem("faults/badMagnet", () ->
                // obj.getFault_BadMagnet().getValue(), LogType.DATALOG),
                // new BooleanLogItem("faults/bootDuringEnable", () ->
                // obj.getFault_BootDuringEnable().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/hardware", () ->
                // obj.getFault_Hardware().getValue(), LogType.DATALOG),
                // new BooleanLogItem("faults/undervoltage", () ->
                // obj.getFault_Undervoltage().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/unlicensed", () ->
                // obj.getFault_UnlicensedFeatureInUse().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/sticky/badMagnet", () ->
                // obj.getStickyFault_BadMagnet().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/sticky/bootDuringEnable",
                // () -> obj.getStickyFault_BootDuringEnable().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/sticky/hardware", () ->
                // obj.getStickyFault_Hardware().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/sticky/undervoltage", () ->
                // obj.getStickyFault_Undervoltage().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/sticky/unlicensed",
                // () -> obj.getStickyFault_UnlicensedFeatureInUse().getValue(),
                // LogType.DATALOG),
        };
    }

    /**
     * Builds CANSparkMax log items.
     * 
     * @param obj the CANSparkMax object to use
     * @return the array of LogItems
     */
    public static AbstractLogItem<?>[] buildSparkAbsoluteEncoderLogItems(SparkAbsoluteEncoder obj) {
        return new AbstractLogItem<?>[] {
                new DoubleLogItem("position", obj::getPosition, LogType.NT),
                new DoubleLogItem("velocity", obj::getVelocity, LogType.NT),
                new DoubleLogItem("zeroOffset", obj::getZeroOffset, LogType.NT),
                new BooleanLogItem("isInverted", obj::getInverted, LogType.DATALOG),
                new DoubleLogItem("positionConversionFactor", obj::getPositionConversionFactor, LogType.DATALOG),
                new DoubleLogItem("velocityConversionFactor", obj::getVelocityConversionFactor, LogType.DATALOG),
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
                new StringLogItem("firmwareVersion", obj::getFirmwareVersion, LogType.DATALOG)
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
                new DoubleLogItem("pitch", () -> obj.getPitch().getValue(), LogType.NT),
                new DoubleLogItem("roll", () -> obj.getRoll().getValue(), LogType.NT),
                new DoubleLogItem("yaw", () -> obj.getYaw().getValue(), LogType.NT),
                new DoubleLogItem("yawRad", () -> Units.degreesToRadians(obj.getYaw().getValue()), LogType.NT),
                new DoubleLogItem("xAcceleration", () -> obj.getAccelerationX().getValue(), LogType.DATALOG),
                new DoubleLogItem("yAcceleration", () -> obj.getAccelerationY().getValue(), LogType.DATALOG),
                new DoubleLogItem("zAcceleration", () -> obj.getAccelerationZ().getValue(), LogType.DATALOG),
                new DoubleLogItem("temperature", () -> obj.getTemperature().getValue(), LogType.DATALOG),
                new IntegerLogItem("deviceId", obj::getDeviceID, LogType.DATALOG),
                // new BooleanLogItem("faults/bootDuringEnable", () ->
                // obj.getFault_BootDuringEnable().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/bootIntoMotion", () ->
                // obj.getFault_BootIntoMotion().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/bootupAccelerometer", () ->
                // obj.getFault_BootupAccelerometer().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/bootupGyroscope", () ->
                // obj.getFault_BootupGyroscope().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/bootupMagnetometer", () ->
                // obj.getFault_BootupMagnetometer().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/dataAcquiredLate", () ->
                // obj.getFault_DataAcquiredLate().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/hardware", () ->
                // obj.getFault_Hardware().getValue(), LogType.DATALOG),
                // new BooleanLogItem("faults/loopTimeSlow", () ->
                // obj.getFault_LoopTimeSlow().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/saturatedAccelerometer",
                // () -> obj.getFault_SaturatedAccelometer().getValue(), LogType.DATALOG),
                // new BooleanLogItem("faults/saturatedGyroscope", () ->
                // obj.getFault_SaturatedGyroscope().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/saturatedMagnetometer",
                // () -> obj.getFault_SaturatedMagnetometer().getValue(), LogType.DATALOG),
                // new BooleanLogItem("faults/undervoltage", () ->
                // obj.getFault_Undervoltage().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/unlicensed", () ->
                // obj.getFault_UnlicensedFeatureInUse().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/sticky/bootDuringEnable",
                // () -> obj.getStickyFault_BootDuringEnable().getValue(), LogType.DATALOG),
                // new BooleanLogItem("faults/sticky/bootIntoMotion", () ->
                // obj.getStickyFault_BootIntoMotion().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/sticky/bootupAccelerometer",
                // () -> obj.getStickyFault_BootupAccelerometer().getValue(), LogType.DATALOG),
                // new BooleanLogItem("faults/sticky/bootupGyroscope",
                // () -> obj.getStickyFault_BootupGyroscope().getValue(), LogType.DATALOG),
                // new BooleanLogItem("faults/sticky/bootupMagnetometer",
                // () -> obj.getStickyFault_BootupMagnetometer().getValue(), LogType.DATALOG),
                // new BooleanLogItem("faults/sticky/dataAcquiredLate",
                // () -> obj.getStickyFault_DataAcquiredLate().getValue(), LogType.DATALOG),
                // new BooleanLogItem("faults/sticky/hardware", () ->
                // obj.getStickyFault_Hardware().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/sticky/loopTimeSlow", () ->
                // obj.getStickyFault_LoopTimeSlow().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/sticky/saturatedAccelerometer",
                // () -> obj.getStickyFault_SaturatedAccelometer().getValue(), LogType.DATALOG),
                // new BooleanLogItem("faults/sticky/saturatedGyroscope",
                // () -> obj.getStickyFault_SaturatedGyroscope().getValue(), LogType.DATALOG),
                // new BooleanLogItem("faults/sticky/saturatedMagnetometer",
                // () -> obj.getStickyFault_SaturatedMagnetometer().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/sticky/undervoltage", () ->
                // obj.getStickyFault_Undervoltage().getValue(),
                // LogType.DATALOG),
                // new BooleanLogItem("faults/sticky/unlicensed",
                // () -> obj.getStickyFault_UnlicensedFeatureInUse().getValue(),
                // LogType.DATALOG),

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
    public static AbstractLogItem<?>[] buildPDHLogItems(PowerDistribution obj) {
        return new AbstractLogItem<?>[] {
                new DoubleLogItem("voltage", obj::getVoltage, LogType.NT),
                new DoubleLogItem("temperature", obj::getTemperature, LogType.NT),
                new DoubleLogItem("totalCurrentAmps", obj::getTotalCurrent, LogType.NT),
                new DoubleLogItem("totalPowerWatts", obj::getTotalPower, LogType.NT),
                new DoubleLogItem("totalEnergyJoules", obj::getTotalEnergy, LogType.NT),
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
                new StringLogItem("firmwareVersion",
                        () -> obj.getVersion().firmwareMajor + "."
                                + obj.getVersion().firmwareMinor + "."
                                + obj.getVersion().firmwareFix,
                        LogType.DATALOG),
                new StringLogItem("type",
                        () -> obj.getType().toString(),
                        LogType.DATALOG),
                new BooleanLogItem("isSwitchableChannelActive", obj::getSwitchableChannel,
                        LogType.NT),
                new BooleanArrayLogItem("faults/breakerFaults",
                        () -> new boolean[] {
                                obj.getFaults().Channel0BreakerFault,
                                obj.getFaults().Channel1BreakerFault,
                                obj.getFaults().Channel2BreakerFault,
                                obj.getFaults().Channel3BreakerFault,
                                obj.getFaults().Channel4BreakerFault,
                                obj.getFaults().Channel5BreakerFault,
                                obj.getFaults().Channel6BreakerFault,
                                obj.getFaults().Channel7BreakerFault,
                                obj.getFaults().Channel8BreakerFault,
                                obj.getFaults().Channel9BreakerFault,
                                obj.getFaults().Channel10BreakerFault,
                                obj.getFaults().Channel11BreakerFault,
                                obj.getFaults().Channel12BreakerFault,
                                obj.getFaults().Channel13BreakerFault,
                                obj.getFaults().Channel14BreakerFault,
                                obj.getFaults().Channel15BreakerFault,
                                obj.getFaults().Channel16BreakerFault,
                                obj.getFaults().Channel17BreakerFault,
                                obj.getFaults().Channel18BreakerFault,
                                obj.getFaults().Channel19BreakerFault,
                                obj.getFaults().Channel20BreakerFault,
                                obj.getFaults().Channel21BreakerFault,
                                obj.getFaults().Channel22BreakerFault,
                                obj.getFaults().Channel23BreakerFault,
                        },
                        LogType.DATALOG),
                new BooleanLogItem("faults/brownout", () -> obj.getFaults().Brownout, LogType.DATALOG),
                new BooleanLogItem("faults/canWarning",
                        () -> obj.getFaults().CanWarning, LogType.DATALOG),
                new BooleanLogItem("faults/hardwareFault", () -> obj.getFaults().HardwareFault,
                        LogType.DATALOG),
                new BooleanArrayLogItem("faults/sticky/breakerFaults",
                        () -> new boolean[] {
                                obj.getStickyFaults().Channel0BreakerFault,
                                obj.getStickyFaults().Channel1BreakerFault,
                                obj.getStickyFaults().Channel2BreakerFault,
                                obj.getStickyFaults().Channel3BreakerFault,
                                obj.getStickyFaults().Channel4BreakerFault,
                                obj.getStickyFaults().Channel5BreakerFault,
                                obj.getStickyFaults().Channel6BreakerFault,
                                obj.getStickyFaults().Channel7BreakerFault,
                                obj.getStickyFaults().Channel8BreakerFault,
                                obj.getStickyFaults().Channel9BreakerFault,
                                obj.getStickyFaults().Channel10BreakerFault,
                                obj.getStickyFaults().Channel11BreakerFault,
                                obj.getStickyFaults().Channel12BreakerFault,
                                obj.getStickyFaults().Channel13BreakerFault,
                                obj.getStickyFaults().Channel14BreakerFault,
                                obj.getStickyFaults().Channel15BreakerFault,
                                obj.getStickyFaults().Channel16BreakerFault,
                                obj.getStickyFaults().Channel17BreakerFault,
                                obj.getStickyFaults().Channel18BreakerFault,
                                obj.getStickyFaults().Channel19BreakerFault,
                                obj.getStickyFaults().Channel20BreakerFault,
                                obj.getStickyFaults().Channel21BreakerFault,
                                obj.getStickyFaults().Channel22BreakerFault,
                                obj.getStickyFaults().Channel23BreakerFault,
                        },
                        LogType.DATALOG),
                new BooleanLogItem("faults/sticky/brownout", () -> obj.getStickyFaults().Brownout,
                        LogType.DATALOG),
                new BooleanLogItem("faults/sticky/canBusOff",
                        () -> obj.getStickyFaults().CanBusOff, LogType.DATALOG),
                new BooleanLogItem("faults/sticky/canWarning",
                        () -> obj.getStickyFaults().CanWarning, LogType.DATALOG),
                new BooleanLogItem("faults/sticky/hasReset", () -> obj.getStickyFaults().HasReset,
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
                new StringLogItem("firmwareVersion",
                        () -> obj.getVersion().firmwareMajor + "."
                                + obj.getVersion().firmwareMinor + "."
                                + obj.getVersion().firmwareFix,
                        LogType.DATALOG),
                new BooleanLogItem("faults/brownout", () -> obj.getFaults().Brownout, LogType.NT),
                new BooleanLogItem("faults/canWarning", () -> obj.getFaults().CanWarning,
                        LogType.DATALOG),
                new BooleanArrayLogItem("faults/channelFaults",
                        () -> new boolean[] {
                                obj.getFaults().Channel0Fault,
                                obj.getFaults().Channel1Fault,
                                obj.getFaults().Channel2Fault,
                                obj.getFaults().Channel3Fault,
                                obj.getFaults().Channel4Fault,
                                obj.getFaults().Channel5Fault,
                                obj.getFaults().Channel6Fault,
                                obj.getFaults().Channel7Fault,
                                obj.getFaults().Channel8Fault,
                                obj.getFaults().Channel9Fault,
                                obj.getFaults().Channel10Fault,
                                obj.getFaults().Channel11Fault,
                                obj.getFaults().Channel12Fault,
                                obj.getFaults().Channel13Fault,
                                obj.getFaults().Channel14Fault,
                                obj.getFaults().Channel15Fault,
                        },
                        LogType.DATALOG),
                new BooleanLogItem("faults/compressorOpen", () -> obj.getFaults().CompressorOpen, LogType.NT),
                new BooleanLogItem("faults/compressorOverCurrent",
                        () -> obj.getFaults().CompressorOverCurrent, LogType.NT),
                new BooleanLogItem("faults/hardwareFault", () -> obj.getFaults().HardwareFault, LogType.NT),
                new BooleanLogItem("faults/solenoidOverCurrent",
                        () -> obj.getFaults().SolenoidOverCurrent, LogType.DATALOG),
                new BooleanLogItem("faults/sticky/brownout", () -> obj.getStickyFaults().Brownout, LogType.DATALOG),
                new BooleanLogItem("faults/sticky/canBusOff",
                        () -> obj.getStickyFaults().CanBusOff, LogType.DATALOG),
                new BooleanLogItem("faults/sticky/canWarning",
                        () -> obj.getStickyFaults().CanWarning, LogType.DATALOG),
                new BooleanLogItem("faults/sticky/compressorOpen",
                        () -> obj.getStickyFaults().CompressorOpen, LogType.DATALOG),
                new BooleanLogItem("faults/sticky/compressorOverCurrent",
                        () -> obj.getStickyFaults().CompressorOverCurrent, LogType.DATALOG),
                new BooleanLogItem("faults/sticky/hasReset", () -> obj.getStickyFaults().HasReset, LogType.DATALOG),
                new BooleanLogItem("faults/sticky/solenoidOverCurrent",
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
                new DoubleLogItem("setpoint", () -> obj.getSetpoint(),
                        LogType.NT),
                new BooleanLogItem("atSetpoint", () -> obj.atSetpoint(),
                        LogType.NT),
                new BooleanLogItem("isContinuousInputEnabled", () -> obj.isContinuousInputEnabled(),
                        LogType.NT),
                new DoubleLogItem("positionError", () -> obj.getPositionError(), LogType.NT),
                new DoubleLogItem("velocityError", () -> obj.getVelocityError(), LogType.DATALOG),
                new TunableDouble("tunables/kP", obj.getP(), (d) -> obj.setP(d)),
                new TunableDouble("tunables/kI", obj.getI(), (d) -> obj.setI(d)),
                new TunableDouble("tunables/kD", obj.getD(), (d) -> obj.setD(d)),
                new TunableDouble("tunables/positionTolerance", obj.getPositionTolerance(),
                        (d) -> obj.setTolerance(d)),
                new TunableDouble("tunables/velocityTolerance", obj.getVelocityTolerance(),
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
                new DoubleLogItem("velocityError", () -> obj.getVelocityError(),
                        LogType.DATALOG),
                new TunableDouble("tunables/kP", obj.getP(), (d) -> obj.setP(d)),
                new TunableDouble("tunables/kI", obj.getI(), (d) -> obj.setI(d)),
                new TunableDouble("tunables/kD", obj.getD(), (d) -> obj.setD(d)),
                new TunableDouble("tunables/positionTolerance", obj.getPositionTolerance(),
                        (d) -> obj.setTolerance(d)),
                new TunableDouble("tunables/velocityTolerance", obj.getVelocityTolerance(),
                        (d) -> obj.setTolerance(obj.getPositionTolerance(), d)),
                new TunableDouble("tunables/velocityConstraint",
                        getConstraints(obj).maxVelocity,
                        (d) -> obj.setConstraints(
                                new TrapezoidProfile.Constraints(d, getConstraints(obj).maxAcceleration))),
                new TunableDouble("tunables/accelerationConstraint",
                        getConstraints(obj).maxAcceleration,
                        (d) -> obj.setConstraints(
                                new TrapezoidProfile.Constraints(getConstraints(obj).maxVelocity, d))),
        };
    }

    public static AbstractLogItem<?>[] buildDCMotorSimLogItems(DCMotorSim obj) {
        return new AbstractLogItem<?>[] {
                new DoubleLogItem("angularPositionRad", obj::getAngularPositionRad, LogType.NT),
                new DoubleLogItem("angularPositionRotations", obj::getAngularPositionRotations, LogType.NT),
                new DoubleLogItem("angularVelocityRadPerSec", obj::getAngularVelocityRadPerSec, LogType.NT),
                new DoubleLogItem("angularVelocityRPM", obj::getAngularVelocityRPM, LogType.NT),
                new DoubleLogItem("currentDrawAmps", obj::getCurrentDrawAmps, LogType.NT),
        };
    }

    public static AbstractLogItem<?>[] buildSingleJointedArmSimLogItems(SingleJointedArmSim obj) {
        return new AbstractLogItem<?>[] {
                new DoubleLogItem("angleRad", obj::getAngleRads, LogType.NT),
                new DoubleLogItem("velocityRadPerSec", obj::getVelocityRadPerSec, LogType.NT),
                new DoubleLogItem("currentDrawAmps", obj::getCurrentDrawAmps, LogType.NT),
        };
    }
}
