package com.techhounds.houndutil.houndlib;

import java.util.ArrayList;
import java.util.function.Function;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class SparkMaxConfigurator {
    private static final ArrayList<CANSparkMax> sparkMaxes = new ArrayList<>();
    public static int MAX_ATTEMPTS = 5;

    /**
     * Creates a brushless CANSparkMax and configures it from factory defaults.
     *
     * <p>
     * All SPARK MAX devices should be created using this method so they can all be
     * stored
     * in a static list.
     *
     * <p>
     * Configuration is applied repeatedly until the attempts exceed MAX_ATTEMPTS.
     *
     * @param id     The CAN ID of the SPARK MAX.
     * @param config All configuration to be applied to the SPARK MAX.
     * @return A CANSparkMax instance.
     * @see #safeBurnFlash()
     */
    @SafeVarargs
    public static CANSparkMax create(int id, MotorType motorType, boolean inverted,
            Function<CANSparkMax, REVLibError>... configs) {
        CANSparkMax sparkMax = new CANSparkMax(id, motorType);

        sparkMax.restoreFactoryDefaults();
        sparkMax.setCANTimeout(0);

        for (Function<CANSparkMax, REVLibError> config : configs) {
            int attempt = 0;

            do {
                REVLibError status = config.apply(sparkMax);
                if (status == REVLibError.kOk) {
                    break;
                } else {
                    DriverStation.reportError(
                            "Failed to configure SPARK MAX [" + id + "]. Error: " + status.toString() + "Attempt #"
                                    + attempt + "/" + MAX_ATTEMPTS,
                            false);
                }
                attempt++;
            } while (attempt <= MAX_ATTEMPTS);

            if (sparkMax.getLastError() != REVLibError.kOk) {
                DriverStation.reportError(
                        "Failed to configure SPARK MAX [" + id + "]: Error: " + sparkMax.getLastError().toString(),
                        false);
            }
        }

        sparkMax.setInverted(inverted); // returns void so no error

        sparkMaxes.add(sparkMax);
        return sparkMax;
    }

    /**
     * Burn all motor configs to flash at the same time, accounting for CAN bus
     * delay. Use once after
     * fully configuring motors.
     */
    public static void safeBurnFlash() {
        Timer.delay(0.2);
        for (CANSparkMax spark : sparkMaxes) {
            spark.burnFlash();
            Timer.delay(0.025);
        }
        Timer.delay(0.2);
    }
}
