package com.techhounds.houndutil.houndlib;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBaseExtensions;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/**
 * A utility class for configuring SPARK MAX and SPARK Flex motor controllers.
 * 
 * <p>
 * 
 * The default SPARK API does not provide a way to retry configuration commands
 * if they fail. Most commands return status codes, but some do not. (see:
 * {@link CANSparkBase#setInverted(boolean)}). This class retries configuration
 * commands that fail, and uses {@link CANSparkBaseExtensions} to receive status
 * codes for other configuration commands. If a failure is detected, it is
 * reported to the Driver Station console.
 * 
 * <p>
 * 
 * SPARK devices have a longstanding bug where burning flash on one device can
 * result in configuration messages to other devices sent directly after the
 * burn to not be received. This class sends all configuration messages
 * initially, and provides a way to safely burn flash on each registered SPARK.
 * 
 * <p>
 * 
 * To use, call one of the create methods to create a SPARK MAX or SPARK Flex,
 * and provide a list of lambda expressions to run to configure the device.
 * After all constructors are called, call
 * {@link SparkConfigurator#safeBurnFlash()} to burn all configurations to
 * flash.
 */
public class SparkConfigurator {
    private static final ArrayList<CANSparkBase> sparks = new ArrayList<>();
    public static int MAX_ATTEMPTS = 5;

    /**
     * Create a SPARK Flex with the given ID, motor type, and configuration
     * parameters. Do not include a call to
     * {@link CANSparkBase#setInverted(boolean)} in the configuration list, specify
     * it in the method call directly.
     * 
     * <p>
     * 
     * This method automatically resets the SPARK Flex to factory defaults.
     * 
     * @param id        the ID of the motor controller
     * @param motorType the type of motor controller (Brushed or Brushless)
     * @param inverted  whether the motor controller is inverted
     * @param configs   a list of lambdas that take in a CANSparkBase to configure a
     *                  specific part of the device
     * @return the fully configured SPARK Flex object
     */
    @SafeVarargs
    public static CANSparkFlex createSparkFlex(int id, MotorType motorType, boolean inverted,
            Function<CANSparkBase, REVLibError>... configs) {
        CANSparkFlex sparkFlex = new CANSparkFlex(id, motorType);
        configure(sparkFlex, inverted, configs);
        return sparkFlex;
    }

    /**
     * Create a SPARK MAX with the given ID, motor type, and configuration
     * parameters. Do not include a call to
     * {@link CANSparkBase#setInverted(boolean)} in the configuration list, specify
     * it in the method call directly.
     * 
     * <p>
     * 
     * This method automatically resets the SPARK MAX to factory defaults.
     * 
     * @param id        the ID of the motor controller
     * @param motorType the type of motor controller (Brushed or Brushless)
     * @param inverted  whether the motor controller is inverted
     * @param configs   a list of lambdas that take in a CANSparkBase to configure a
     *                  specific part of the device
     * @return the fully configured SPARK MAX object
     */
    @SafeVarargs
    public static CANSparkMax createSparkMax(int id, MotorType motorType, boolean inverted,
            Function<CANSparkBase, REVLibError>... configs) {
        CANSparkMax sparkMax = new CANSparkMax(id, motorType);
        configure(sparkMax, inverted, configs);
        return sparkMax;
    }

    /**
     * Configures a SPARK MAX or SPARK Flex with the given configuration parameters.
     * Retries configuration commands that fail a maximum of {@code MAX_ATTEMPTS}
     * times (5).
     * 
     * @param motor
     * @param inverted
     * @param configs
     */
    @SafeVarargs
    private static void configure(CANSparkBase motor, boolean inverted,
            Function<CANSparkBase, REVLibError>... configs) {

        ArrayList<Function<CANSparkBase, REVLibError>> configsArr = new ArrayList<>(List.of(configs));
        if (inverted) {
            configsArr.add((s) -> CANSparkBaseExtensions.setInverted(s, inverted));
        }
        motor.restoreFactoryDefaults();
        motor.setCANTimeout(250);

        for (Function<CANSparkBase, REVLibError> config : configsArr) {
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
        sparks.add(motor);
    }

    /**
     * Burn all motor configs to flash at the same time, accounting for CAN bus
     * delay. Use once after fully configuring motors.
     */
    public static void safeBurnFlash() {
        Timer.delay(0.2);
        for (CANSparkBase spark : sparks) {
            spark.burnFlash();
            Timer.delay(0.025);
        }
        Timer.delay(0.2);
    }

    /**
     * Static method to return a formatted name for a SPARK Flex.
     * 
     * @param spark the SPARK Flex to name
     * @return the formatted name
     */
    public static String name(CANSparkFlex spark) {
        return "SPARK Flex [" + spark.getDeviceId() + "]";
    }

    /**
     * Static method to return a formatted name for a SPARK MAX.
     * 
     * @param spark the SPARK MAX to name
     * @return the formatted name
     */
    public static String name(CANSparkMax spark) {
        return "SPARK MAX [" + spark.getDeviceId() + "]";
    }
}
