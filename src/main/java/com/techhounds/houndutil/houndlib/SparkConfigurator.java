package com.techhounds.houndutil.houndlib;

import java.util.ArrayList;
import java.util.function.Function;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class SparkConfigurator {
    private static final ArrayList<CANSparkBase> sparks = new ArrayList<>();
    public static int MAX_ATTEMPTS = 5;

    @SafeVarargs
    public static CANSparkFlex createSparkFlex(int id, MotorType motorType, boolean inverted,
            Function<CANSparkBase, REVLibError>... configs) {
        CANSparkFlex sparkFlex = new CANSparkFlex(id, motorType);
        configure(sparkFlex, inverted, configs);
        return sparkFlex;
    }

    @SafeVarargs
    public static CANSparkMax createSparkMax(int id, MotorType motorType, boolean inverted,
            Function<CANSparkBase, REVLibError>... configs) {
        CANSparkMax sparkMax = new CANSparkMax(id, motorType);
        configure(sparkMax, inverted, configs);
        return sparkMax;
    }

    @SafeVarargs
    private static void configure(CANSparkBase motor, boolean inverted,
            Function<CANSparkBase, REVLibError>... configs) {
        motor.restoreFactoryDefaults();
        motor.setCANTimeout(250);

        for (Function<CANSparkBase, REVLibError> config : configs) {
            int attempt = 0;

            do {
                REVLibError status = config.apply(motor);
                if (status == REVLibError.kOk) {
                    break;
                } else {
                    DriverStation.reportError(
                            "Failed to configure SPARK MAX [" + motor.getDeviceId() + "]. Error: " + status.toString()
                                    + "Attempt #"
                                    + attempt + "/" + MAX_ATTEMPTS,
                            false);
                }
                attempt++;
            } while (attempt <= MAX_ATTEMPTS);

            if (motor.getLastError() != REVLibError.kOk) {
                DriverStation.reportError(
                        "Failed to configure SPARK MAX [" + motor.getDeviceId() + "]: Error: "
                                + motor.getLastError().toString(),
                        false);
            }
        }

        motor.setCANTimeout(0);
        motor.setInverted(inverted); // returns void so no error

        sparks.add(motor);
    }

    /**
     * Burn all motor configs to flash at the same time, accounting for CAN bus
     * delay. Use once after
     * fully configuring motors.
     */
    public static void safeBurnFlash() {
        Timer.delay(0.2);
        for (CANSparkBase spark : sparks) {
            spark.burnFlash();
            Timer.delay(0.025);
        }
        Timer.delay(0.2);
    }
}
