package com.techhounds.houndutil.houndlog;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.hal.PowerDistributionStickyFaults;
import edu.wpi.first.hal.REVPHFaults;
import edu.wpi.first.hal.REVPHStickyFaults;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;

/**
 * Provides a utility that can be shown on a dashboard that displays faults on
 * registered devices.
 * 
 * To use, add the object as an Alerts widget in Elastic.
 */
public final class FaultLogger {
    /** An individual fault, containing a name and description. */
    public static record Fault(String name, String description, FaultType type) {
        @Override
        public String toString() {
            return name + ": " + description;
        }
    }

    /**
     * The type of fault, used for displaying an icon next to the fault displayed in
     * the Alerts widget.
     */
    public static enum FaultType {
        /**
         * Indicates that a device isn't failing, but there is something that a
         * programmer or driver may want to know (i.e. FMS connected).
         */
        INFO,
        /**
         * Indicates an issue that is not critical, but might impact performance or need
         * attention (i.e. motor temperature approaching limit, CAN utilization high).
         */
        WARNING,
        /**
         * Indicates an issue that prevents normal operation and requires immediate
         * attention (i.e. motor controller disconnected, sensor disconnected, breaker
         * tripped).
         */
        ERROR,
    }

    /**
     * Represents an alert widget on NetworkTables. Active faults are automatically
     * added to this widget, and are sent over NetworkTables. This class should not
     * be instantiated directly.
     */
    public static class Alerts {
        private final StringArrayPublisher errors;
        private final StringArrayPublisher warnings;
        private final StringArrayPublisher infos;

        /**
         * Creates an Alerts widget.
         * 
         * @param base the base NetworkTable to add this widget to
         * @param name the name of the widget
         */
        public Alerts(NetworkTable base, String name) {
            NetworkTable table = base.getSubTable(name);
            table.getStringTopic(".type").publish().set("Alerts");
            errors = table.getStringArrayTopic("errors").publish();
            warnings = table.getStringArrayTopic("warnings").publish();
            infos = table.getStringArrayTopic("infos").publish();
        }

        /**
         * Sets this Alerts widget to a given set of faults.
         * 
         * @param faults the faults to publish
         */
        public void set(Set<Fault> faults) {
            errors.set(filteredStrings(faults, FaultType.ERROR));
            warnings.set(filteredStrings(faults, FaultType.WARNING));
            infos.set(filteredStrings(faults, FaultType.INFO));
        }
    }

    private static final List<Supplier<Optional<Fault>>> faultSuppliers = new ArrayList<>();
    private static final Set<Fault> newFaults = new HashSet<>();
    private static final Set<Fault> activeFaults = new HashSet<>();
    private static final Set<Fault> totalFaults = new HashSet<>();

    private static final NetworkTable base = NetworkTableInstance.getDefault().getTable("HoundLog/faultLogger");
    private static final Alerts activeAlerts = new Alerts(base, "activeFaults");
    private static final Alerts totalAlerts = new Alerts(base, "totalFaults");

    /** Polls registered fallibles. This method should be called periodically. */
    public static void update() {
        activeFaults.clear();

        faultSuppliers.stream()
                .map(s -> s.get())
                .flatMap(Optional::stream)
                .forEach(FaultLogger::report);

        activeFaults.addAll(newFaults);
        newFaults.clear();

        totalFaults.addAll(activeFaults);

        activeAlerts.set(activeFaults);
        totalAlerts.set(totalFaults);
    }

    /** Clears total faults. */
    public static void clear() {
        totalFaults.clear();
    }

    /** Clears fault suppliers. */
    public static void unregisterAll() {
        faultSuppliers.clear();
    }

    /**
     * Returns the set of all current faults.
     *
     * @return the set of all current faults
     */
    public static Set<Fault> activeFaults() {
        return activeFaults;
    }

    /**
     * Returns the set of all total faults.
     *
     * @return the set of all total faults
     */
    public static Set<Fault> totalFaults() {
        return totalFaults;
    }

    /**
     * Reports a fault. Call this periodically to display this fault over an
     * extended period of time.
     *
     * @param fault the fault to report
     */
    public static void report(Fault fault) {
        newFaults.add(fault);
    }

    /**
     * Reports a fault.
     *
     * @param name        the name of the fault
     * @param description the description of the fault
     * @param type        the type of the fault
     */
    public static void report(String name, String description, FaultType type) {
        report(new Fault(name, description, type));
    }

    /**
     * Registers a new fault supplier.
     *
     * @param supplier a supplier that can provide a fault
     */
    public static void register(Supplier<Optional<Fault>> supplier) {
        faultSuppliers.add(supplier);
    }

    /**
     * Registers a new fault supplier.
     *
     * @param condition   whether a failure is occuring
     * @param description the failure's description
     * @param type        the type of failure.
     */
    public static void register(
            BooleanSupplier condition, String name, String description, FaultType type) {
        faultSuppliers.add(
                () -> condition.getAsBoolean()
                        ? Optional.of(new Fault(name, description, type))
                        : Optional.empty());
    }

    /**
     * Registers fault suppliers for a SPARK Flex.
     *
     * @param spark the SPARK Flex to register
     */
    public static void register(SparkFlex spark) {
        Map<String, BooleanSupplier> faults = new LinkedHashMap<>(Map.of(
                "Motor Type Fault", () -> spark.getFaults().motorType,
                "Sensor Fault", () -> spark.getFaults().sensor,
                "CAN Fault", () -> spark.getFaults().can,
                "Temperature Fault", () -> spark.getFaults().temperature,
                "Gate Driver Fault", () -> spark.getFaults().gateDriver,
                "ESC Fault", () -> spark.getFaults().escEeprom,
                "Firmware Fault", () -> spark.getFaults().firmware));
        for (String faultName : faults.keySet()) {
            register(faults.get(faultName), "SPARK Flex [" + spark.getDeviceId() + "]", faultName,
                    FaultType.ERROR);
        }
        register(
                () -> spark.getMotorTemperature() > 80,
                "SPARK Flex [" + spark.getDeviceId() + "]",
                "motor above 80°C",
                FaultType.WARNING);

    }

    /**
     * Registers fault suppliers for a SPARK MAX.
     *
     * @param spark the SPARK MAX to register
     */
    public static void register(SparkMax spark) {
        Map<String, BooleanSupplier> faults = new LinkedHashMap<>(Map.of(
                "Motor Type Fault", () -> spark.getFaults().motorType,
                "Sensor Fault", () -> spark.getFaults().sensor,
                "CAN Fault", () -> spark.getFaults().can,
                "Temperature Fault", () -> spark.getFaults().temperature,
                "Gate Driver Fault", () -> spark.getFaults().gateDriver,
                "ESC Fault", () -> spark.getFaults().escEeprom,
                "Firmware Fault", () -> spark.getFaults().firmware));
        for (String faultName : faults.keySet()) {
            register(faults.get(faultName), "SPARK MAX [" + spark.getDeviceId() + "]", faultName,
                    FaultType.ERROR);
        }
        register(
                () -> spark.getMotorTemperature() > 80,
                "SPARK MAX [" + spark.getDeviceId() + "]",
                "motor above 80°C",
                FaultType.WARNING);
    }

    /**
     * Registers fault suppliers for a Talon FX.
     *
     * @param talon the Talon FX to register
     */
    public static void register(TalonFX talon) {
        List<StatusSignal<Boolean>> faultSignals = List.of(
                talon.getFault_BootDuringEnable(),
                talon.getFault_BridgeBrownout(),
                talon.getFault_DeviceTemp(),
                talon.getFault_ForwardHardLimit(),
                talon.getFault_ForwardSoftLimit(),
                talon.getFault_FusedSensorOutOfSync(),
                talon.getFault_Hardware(),
                talon.getFault_MissingDifferentialFX(),
                talon.getFault_OverSupplyV(),
                talon.getFault_ProcTemp(),
                talon.getFault_RemoteSensorDataInvalid(),
                talon.getFault_RemoteSensorPosOverflow(),
                talon.getFault_RemoteSensorReset(),
                talon.getFault_ReverseHardLimit(),
                talon.getFault_ReverseSoftLimit(),
                talon.getFault_StatorCurrLimit(),
                talon.getFault_SupplyCurrLimit(),
                talon.getFault_Undervoltage(),
                talon.getFault_UnlicensedFeatureInUse(),
                talon.getFault_UnstableSupplyV(),
                talon.getStickyFault_UsingFusedCANcoderWhileUnlicensed(),
                talon.getStickyFault_BootDuringEnable(),
                talon.getStickyFault_BridgeBrownout(),
                talon.getStickyFault_DeviceTemp(),
                talon.getStickyFault_ForwardHardLimit(),
                talon.getStickyFault_ForwardSoftLimit(),
                talon.getStickyFault_FusedSensorOutOfSync(),
                talon.getStickyFault_Hardware(),
                talon.getStickyFault_MissingDifferentialFX(),
                talon.getStickyFault_OverSupplyV(),
                talon.getStickyFault_ProcTemp(),
                talon.getStickyFault_RemoteSensorDataInvalid(),
                talon.getStickyFault_RemoteSensorPosOverflow(),
                talon.getStickyFault_RemoteSensorReset(),
                talon.getStickyFault_ReverseHardLimit(),
                talon.getStickyFault_ReverseSoftLimit(),
                talon.getStickyFault_StatorCurrLimit(),
                talon.getStickyFault_SupplyCurrLimit(),
                talon.getStickyFault_Undervoltage(),
                talon.getStickyFault_UnlicensedFeatureInUse(),
                talon.getStickyFault_UnstableSupplyV(),
                talon.getStickyFault_UsingFusedCANcoderWhileUnlicensed());

        faultSignals.forEach((s) -> SignalManager.register(s));

        for (StatusSignal<Boolean> signal : faultSignals) {
            register(signal::getValue, "Talon FX [" + talon.getDeviceID() + "]", signal.getName(), FaultType.ERROR);
        }
        register(
                () -> talon.getDeviceTemp().getValueAsDouble() > 80,
                "Talon FX [" + talon.getDeviceID() + "]",
                "motor above 80°C",
                FaultType.WARNING);
    }

    /**
     * Registers fault suppliers for a CANcoder.
     *
     * @param talon the CANcoder to register
     */
    public static void register(CANcoder cancoder) {
        List<StatusSignal<Boolean>> faultSignals = List.of(
                cancoder.getFault_BadMagnet(),
                cancoder.getFault_BootDuringEnable(),
                cancoder.getFault_Hardware(),
                cancoder.getFault_Undervoltage(),
                cancoder.getFault_UnlicensedFeatureInUse(),
                cancoder.getStickyFault_BadMagnet(),
                cancoder.getStickyFault_BootDuringEnable(),
                cancoder.getStickyFault_Hardware(),
                cancoder.getStickyFault_Undervoltage(),
                cancoder.getStickyFault_UnlicensedFeatureInUse());

        faultSignals.forEach((s) -> SignalManager.register(s));

        for (StatusSignal<Boolean> signal : faultSignals) {
            register(signal::getValue, "CANcoder [" + cancoder.getDeviceID() + "]", signal.getName(),
                    FaultType.ERROR);
        }
    }

    /**
     * Registers fault suppliers for a Pigeon 2.
     *
     * @param talon the Pigeon 2 to register
     */
    public static void register(Pigeon2 pigeon) {
        List<StatusSignal<Boolean>> faultSignals = List.of(
                pigeon.getFault_BootDuringEnable(),
                pigeon.getFault_BootIntoMotion(),
                pigeon.getFault_BootupAccelerometer(),
                pigeon.getFault_BootupGyroscope(),
                pigeon.getFault_BootupMagnetometer(),
                pigeon.getFault_DataAcquiredLate(),
                pigeon.getFault_Hardware(),
                pigeon.getFault_LoopTimeSlow(),
                pigeon.getFault_SaturatedAccelerometer(),
                pigeon.getFault_SaturatedGyroscope(),
                pigeon.getFault_SaturatedMagnetometer(),
                pigeon.getFault_Undervoltage(),
                pigeon.getFault_UnlicensedFeatureInUse(),
                pigeon.getStickyFault_BootDuringEnable(),
                pigeon.getStickyFault_BootIntoMotion(),
                pigeon.getStickyFault_BootupAccelerometer(),
                pigeon.getStickyFault_BootupGyroscope(),
                pigeon.getStickyFault_BootupMagnetometer(),
                pigeon.getStickyFault_DataAcquiredLate(),
                pigeon.getStickyFault_Hardware(),
                pigeon.getStickyFault_LoopTimeSlow(),
                pigeon.getStickyFault_SaturatedAccelerometer(),
                pigeon.getStickyFault_SaturatedGyroscope(),
                pigeon.getStickyFault_SaturatedMagnetometer(),
                pigeon.getStickyFault_Undervoltage(),
                pigeon.getStickyFault_UnlicensedFeatureInUse());

        faultSignals.forEach((s) -> SignalManager.register(s));

        for (StatusSignal<Boolean> signal : faultSignals) {
            register(signal::getValue, "Pigeon 2 [" + pigeon.getDeviceID() + "]", signal.getName(),
                    FaultType.ERROR);
        }
    }

    /**
     * Registers fault suppliers for a power distribution hub/panel.
     *
     * @param powerDistribution the power distribution device to register
     */
    public static void register(PowerDistribution powerDistribution) {
        powerDistribution.getFaults();
        for (Field field : PowerDistributionFaults.class.getFields()) {
            register(
                    () -> {
                        try {
                            if (field.getBoolean(powerDistribution.getFaults())) {
                                return Optional.of(
                                        new Fault("PDH", field.getName(), FaultType.ERROR));
                            }
                        } catch (Exception e) {
                            e.printStackTrace();
                        }
                        return Optional.empty();
                    });
        }
        for (Field field : PowerDistributionStickyFaults.class.getFields()) {
            register(
                    () -> {
                        try {
                            if (field.getBoolean(powerDistribution.getStickyFaults())) {
                                return Optional.of(
                                        new Fault("PDH", field.getName(), FaultType.ERROR));
                            }
                        } catch (Exception e) {
                        }
                        return Optional.empty();
                    });
        }
    }

    /**
     * Registers fault suppliers for a REV Pneumatic Hub.
     * 
     * @param ph the Pneumatic Hub to register
     */
    public static void register(PneumaticHub ph) {
        for (Field field : REVPHFaults.class.getFields()) {
            register(
                    () -> {
                        try {
                            if (field.getBoolean(ph.getFaults())) {
                                return Optional.of(
                                        new Fault("Pneumatic Hub", field.getName(), FaultType.ERROR));
                            }
                        } catch (Exception e) {
                            e.printStackTrace();
                        }
                        return Optional.empty();
                    });
        }
        for (Field field : REVPHStickyFaults.class.getFields()) {
            register(
                    () -> {
                        try {
                            if (field.getBoolean(ph.getStickyFaults())) {
                                return Optional.of(
                                        new Fault("Pneumatic Hub", field.getName(), FaultType.ERROR));
                            }
                        } catch (Exception e) {
                            e.printStackTrace();
                        }
                        return Optional.empty();
                    });
        }
    }

    /**
     * Registers fault suppliers for a PhotonVision camera.
     *
     * @param camera the camera to register
     */
    public static void register(PhotonCamera camera) {
        register(
                () -> !camera.isConnected(),
                "Photon Camera [" + camera.getName() + "]",
                "disconnected",
                FaultType.ERROR);
    }

    /**
     * Returns an array of descriptions of all faults that match the specified type.
     *
     * @param type The type to filter for.
     * @return An array of description strings.
     */
    private static String[] filteredStrings(Set<Fault> faults, FaultType type) {
        return faults.stream()
                .filter(a -> a.type() == type)
                .map(Fault::toString)
                .toArray(String[]::new);
    }
}