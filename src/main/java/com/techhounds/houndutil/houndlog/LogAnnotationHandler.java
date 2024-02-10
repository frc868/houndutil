package com.techhounds.houndutil.houndlog;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.techhounds.houndutil.houndlog.interfaces.Log;
import com.techhounds.houndutil.houndlog.interfaces.LoggedObject;
import com.techhounds.houndutil.houndlog.interfaces.SendableLog;
import com.techhounds.houndutil.houndlog.interfaces.Tunable;
import com.techhounds.houndutil.houndlog.loggers.Logger;
import com.techhounds.houndutil.houndlog.loggers.SendableLogger;
import com.techhounds.houndutil.houndlog.logitems.BooleanArrayLogItem;
import com.techhounds.houndutil.houndlog.logitems.BooleanLogItem;
import com.techhounds.houndutil.houndlog.logitems.DoubleArrayLogItem;
import com.techhounds.houndutil.houndlog.logitems.DoubleLogItem;
import com.techhounds.houndutil.houndlog.logitems.FloatArrayLogItem;
import com.techhounds.houndutil.houndlog.logitems.FloatLogItem;
import com.techhounds.houndutil.houndlog.logitems.IntegerArrayLogItem;
import com.techhounds.houndutil.houndlog.logitems.IntegerLogItem;
import com.techhounds.houndutil.houndlog.logitems.StringArrayLogItem;
import com.techhounds.houndutil.houndlog.logitems.StringLogItem;
import com.techhounds.houndutil.houndlog.logitems.StructLogItem;
import com.techhounds.houndutil.houndlog.logitems.StructArrayLogItem;
import com.techhounds.houndutil.houndlog.logitems.TunableBoolean;
import com.techhounds.houndutil.houndlog.logitems.TunableDouble;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import com.techhounds.houndutil.houndlog.loggers.DeviceLogger;
import static java.util.Map.entry;

/**
 * Deals with objects that have been annotated with @Log.
 * 
 * Contains default logging configurations for objects like a PIDController.
 */
public class LogAnnotationHandler {
    protected static void handleLoggedObject(Object loggedObject) {
        handleLoggedObject(loggedObject, "", new ArrayList<String>());
    }

    protected static void handleLoggedObject(Object loggedObject, String name, ArrayList<String> subkeys) {
        handleLoggedObjectImpl(
                loggedObject,
                loggedObject.getClass().getDeclaredFields(),
                loggedObject.getClass().getDeclaredMethods(),
                name, subkeys);
    }

    protected static void handleLoggedClass(Class<?> loggedClass, String name, ArrayList<String> subkeys) {
        handleLoggedObjectImpl(
                null,
                loggedClass.getDeclaredFields(),
                loggedClass.getDeclaredMethods(),
                name, subkeys);
    }

    private static void handleLoggedObjectImpl(Object loggedObject, Field[] fields, Method[] methods, String name,
            ArrayList<String> subkeys) {
        ArrayList<Logger> loggers = new ArrayList<Logger>();
        for (Method method : methods) {
            Log subLogAnnotation = method.getAnnotation(Log.class);
            if (subLogAnnotation != null) {
                String varName = subLogAnnotation.name().equals("") ? method.getName() : subLogAnnotation.name();
                // if method is named "getDouble", this would change it to "double"
                if (varName.startsWith("get")) {
                    varName = varName.substring(3, 4).toLowerCase() + varName.substring(4);
                }
                method.setAccessible(true);

                Supplier<Object> valueSupplier = () -> {
                    try {
                        return method.invoke(loggedObject);
                    } catch (Exception e) {
                        e.printStackTrace();
                        return null;
                    }
                };

                Optional<Logger> optLogger = getLoggerForValue(valueSupplier, subLogAnnotation, varName);
                if (optLogger.isPresent()) {
                    loggers.add(optLogger.get());
                }
            }
        }

        for (Field field : fields) {
            field.setAccessible(true);
            Log subLogAnnotation = field.getAnnotation(Log.class);
            if (subLogAnnotation != null) {
                String varName = subLogAnnotation.name().equals("") ? field.getName() : subLogAnnotation.name();
                LoggedObject loggedObjectAnnotation = field.getType().getAnnotation(LoggedObject.class);
                if (loggedObjectAnnotation != null) {
                    try {
                        String[] logGroups = subLogAnnotation.groups();
                        ArrayList<String> updatedSubkeys = new ArrayList<String>(subkeys);
                        if (!name.equals(""))
                            updatedSubkeys.add(name);
                        updatedSubkeys.addAll(Arrays.asList(logGroups));

                        handleLoggedObject(field.get(loggedObject), varName, updatedSubkeys);
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                } else {
                    field.setAccessible(true);

                    Supplier<Object> valueSupplier = () -> {
                        try {
                            return field.get(loggedObject);
                        } catch (Exception e) {
                            e.printStackTrace();
                            return null;
                        }
                    };
                    if (field.getAnnotation(Tunable.class) != null) {
                        Object value = valueSupplier.get();
                        if (value.getClass() == Double.class) {
                            DoubleConsumer consumer = (d) -> {
                                try {
                                    field.set(loggedObject, d);
                                } catch (Exception e) {
                                    e.printStackTrace();
                                }
                            };

                            TunableDouble logger = new TunableDouble(getName(subLogAnnotation, varName),
                                    (double) valueSupplier.get(), consumer);
                            loggers.add(logger);
                        } else if (value.getClass() == Boolean.class) {
                            BooleanConsumer consumer = (d) -> {
                                try {
                                    field.set(loggedObject, d);
                                } catch (Exception e) {
                                    e.printStackTrace();
                                }
                            };

                            TunableBoolean logger = new TunableBoolean(getName(subLogAnnotation, varName),
                                    (boolean) valueSupplier.get(), consumer);
                            loggers.add(logger);
                        }

                    } else {
                        Optional<Logger> optLogger = getLoggerForValue(valueSupplier, subLogAnnotation, varName);
                        if (optLogger.isPresent()) {
                            loggers.add(optLogger.get());
                        }
                    }
                }
            }

            SendableLog subSendableLogAnnotation = field.getAnnotation(SendableLog.class);
            if (subSendableLogAnnotation != null) {
                ArrayList<String> nameComponents = new ArrayList<String>();
                nameComponents.addAll(Arrays.asList(subSendableLogAnnotation.groups()));
                String varName = subSendableLogAnnotation.name().equals("") ? field.getName() : subLogAnnotation.name();
                nameComponents.add(varName);
                String formattedName = String.join("/", nameComponents);

                try {
                    Sendable sendable = (Sendable) field.get(loggedObject);
                    loggers.add(new SendableLogger(formattedName, sendable));
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }

        }
        if (name != null)
            subkeys.add(name);
        LoggingManager.getInstance().addGroup(String.join("/", subkeys),
                new LogGroup(loggers.toArray(new Logger[loggers.size()])));
    }

    public static String getName(Log logAnnotation, String varName) {
        ArrayList<String> nameComponents = new ArrayList<String>();
        nameComponents.addAll(Arrays.asList(logAnnotation.groups()));
        nameComponents.add(varName);
        return String.join("/", nameComponents);
    }

    /**
     * 
     * @param field
     * @param object the object to get the value from
     * @return
     */
    @SuppressWarnings("unchecked")
    private static Optional<Logger> getLoggerForValue(Supplier<Object> valueSupplier, Log logAnnotation,
            String varName) {
        try {
            Supplier<Object> checkedValueSupplier;

            Object value = valueSupplier.get();
            String name = getName(logAnnotation, varName);

            if (value instanceof Supplier) {
                checkedValueSupplier = (Supplier<Object>) value;
                value = ((Supplier<Object>) value).get();
            } else {
                checkedValueSupplier = valueSupplier;
            }

            Map<Class<?>, Supplier<Logger>> classToLoggerMap = Map.ofEntries(
                    entry(boolean[].class,
                            () -> new BooleanArrayLogItem(name,
                                    () -> (boolean[]) checkedValueSupplier.get(), logAnnotation.logLevel())),
                    entry(Boolean.class,
                            () -> new BooleanLogItem(name,
                                    () -> (boolean) checkedValueSupplier.get(), logAnnotation.logLevel())),
                    entry(double[].class,
                            () -> new DoubleArrayLogItem(name,
                                    () -> (double[]) checkedValueSupplier.get(), logAnnotation.logLevel())),
                    entry(Double.class,
                            () -> new DoubleLogItem(name,
                                    () -> (double) checkedValueSupplier.get(), logAnnotation.logLevel())),
                    entry(float[].class,
                            () -> new FloatArrayLogItem(name,
                                    () -> (float[]) checkedValueSupplier.get(), logAnnotation.logLevel())),
                    entry(Float.class,
                            () -> new FloatLogItem(name,
                                    () -> (float) checkedValueSupplier.get(), logAnnotation.logLevel())),
                    entry(int[].class,
                            () -> new IntegerArrayLogItem(name,
                                    () -> (int[]) checkedValueSupplier.get(), logAnnotation.logLevel())),
                    entry(Integer.class,
                            () -> new IntegerLogItem(name,
                                    () -> (int) checkedValueSupplier.get(), logAnnotation.logLevel())),
                    entry(String[].class,
                            () -> new StringArrayLogItem(name,
                                    () -> (String[]) checkedValueSupplier.get(), logAnnotation.logLevel())),
                    entry(String.class,
                            () -> new StringLogItem(name,
                                    () -> (String) checkedValueSupplier.get(), logAnnotation.logLevel())),
                    entry(TalonFX.class,
                            () -> new DeviceLogger(name,
                                    LogProfileBuilder.buildTalonFXLogItems((TalonFX) checkedValueSupplier.get()))),
                    entry(CANSparkMax.class,
                            () -> new DeviceLogger(name,
                                    LogProfileBuilder
                                            .buildCANSparkMaxLogItems((CANSparkMax) checkedValueSupplier.get()))),
                    entry(CANSparkFlex.class,
                            () -> new DeviceLogger(name,
                                    LogProfileBuilder
                                            .buildCANSparkFlexLogItems((CANSparkFlex) checkedValueSupplier.get()))),
                    entry(CANcoder.class,
                            () -> new DeviceLogger(name,
                                    LogProfileBuilder.buildCANcoderLogItems((CANcoder) checkedValueSupplier.get()))),
                    entry(SparkAbsoluteEncoder.class,
                            () -> new DeviceLogger(name,
                                    LogProfileBuilder.buildSparkAbsoluteEncoderLogItems(
                                            (SparkAbsoluteEncoder) checkedValueSupplier.get()))),
                    entry(AHRS.class,
                            () -> new DeviceLogger(name,
                                    LogProfileBuilder.buildNavXLogItems((AHRS) checkedValueSupplier.get()))),
                    entry(Pigeon2.class,
                            () -> new DeviceLogger(name,
                                    LogProfileBuilder.buildPigeon2LogItems((Pigeon2) checkedValueSupplier.get()))),
                    entry(DoubleSolenoid.class,
                            () -> new DeviceLogger(name,
                                    LogProfileBuilder
                                            .buildDoubleSolenoidLogItems((DoubleSolenoid) checkedValueSupplier.get()))),
                    entry(PowerDistribution.class,
                            () -> new DeviceLogger(name,
                                    LogProfileBuilder
                                            .buildPDHLogItems((PowerDistribution) checkedValueSupplier.get()))),
                    entry(PneumaticHub.class,
                            () -> new DeviceLogger(name,
                                    LogProfileBuilder
                                            .buildPneumaticHubLogItems((PneumaticHub) checkedValueSupplier.get()))),
                    entry(PIDController.class,
                            () -> new DeviceLogger(name,
                                    LogProfileBuilder
                                            .buildPIDControllerLogItems((PIDController) checkedValueSupplier.get()))),
                    entry(ProfiledPIDController.class,
                            () -> new DeviceLogger(name,
                                    LogProfileBuilder.buildProfiledPIDControllerLogItems(
                                            (ProfiledPIDController) checkedValueSupplier.get()))),
                    entry(DCMotorSim.class,
                            () -> new DeviceLogger(name,
                                    LogProfileBuilder.buildDCMotorSimLogItems(
                                            (DCMotorSim) checkedValueSupplier.get()))),
                    entry(SingleJointedArmSim.class,
                            () -> new DeviceLogger(name,
                                    LogProfileBuilder.buildSingleJointedArmSimLogItems(
                                            (SingleJointedArmSim) checkedValueSupplier.get()))),
                    entry(DigitalInput.class,
                            () -> new BooleanLogItem(name,
                                    () -> ((DigitalInput) checkedValueSupplier.get()).get())),
                    entry(Pose2d.class,
                            () -> new StructLogItem<Pose2d>(name, Pose2d.struct,
                                    () -> (Pose2d) checkedValueSupplier.get(),
                                    logAnnotation.logLevel())),
                    entry(Pose2d[].class,
                            () -> new StructArrayLogItem<Pose2d>(name, Pose2d.struct,
                                    () -> (Pose2d[]) checkedValueSupplier.get(),
                                    logAnnotation.logLevel())),
                    entry(Pose3d.class,
                            () -> new StructLogItem<Pose3d>(name, Pose3d.struct,
                                    () -> (Pose3d) checkedValueSupplier.get(),
                                    logAnnotation.logLevel())),
                    entry(Pose3d[].class,
                            () -> new StructArrayLogItem<Pose3d>(name, Pose3d.struct,
                                    () -> (Pose3d[]) checkedValueSupplier.get(),
                                    logAnnotation.logLevel())),
                    entry(Rotation2d.class,
                            () -> new StructLogItem<Rotation2d>(name, Rotation2d.struct,
                                    () -> (Rotation2d) checkedValueSupplier.get(),
                                    logAnnotation.logLevel())),
                    entry(Rotation2d[].class,
                            () -> new StructArrayLogItem<Rotation2d>(name, Rotation2d.struct,
                                    () -> (Rotation2d[]) checkedValueSupplier.get(),
                                    logAnnotation.logLevel())),
                    entry(Rotation3d.class,
                            () -> new StructLogItem<Rotation3d>(name, Rotation3d.struct,
                                    () -> (Rotation3d) checkedValueSupplier.get(),
                                    logAnnotation.logLevel())),
                    entry(Rotation3d[].class,
                            () -> new StructArrayLogItem<Rotation3d>(name, Rotation3d.struct,
                                    () -> (Rotation3d[]) checkedValueSupplier.get(),
                                    logAnnotation.logLevel())),
                    entry(Transform2d.class,
                            () -> new StructLogItem<Transform2d>(name, Transform2d.struct,
                                    () -> (Transform2d) checkedValueSupplier.get(),
                                    logAnnotation.logLevel())),
                    entry(Transform2d[].class,
                            () -> new StructArrayLogItem<Transform2d>(name, Transform2d.struct,
                                    () -> (Transform2d[]) checkedValueSupplier.get(),
                                    logAnnotation.logLevel())),
                    entry(Transform3d.class,
                            () -> new StructLogItem<Transform3d>(name, Transform3d.struct,
                                    () -> (Transform3d) checkedValueSupplier.get(),
                                    logAnnotation.logLevel())),
                    entry(Transform3d[].class,
                            () -> new StructArrayLogItem<Transform3d>(name, Transform3d.struct,
                                    () -> (Transform3d[]) checkedValueSupplier.get(),
                                    logAnnotation.logLevel())),
                    entry(Translation2d.class,
                            () -> new StructLogItem<Translation2d>(name, Translation2d.struct,
                                    () -> (Translation2d) checkedValueSupplier.get(),
                                    logAnnotation.logLevel())),
                    entry(Translation2d[].class,
                            () -> new StructArrayLogItem<Translation2d>(name, Translation2d.struct,
                                    () -> (Translation2d[]) checkedValueSupplier.get(),
                                    logAnnotation.logLevel())),
                    entry(Translation3d.class,
                            () -> new StructLogItem<Translation3d>(name, Translation3d.struct,
                                    () -> (Translation3d) checkedValueSupplier.get(),
                                    logAnnotation.logLevel())),
                    entry(Translation3d[].class,
                            () -> new StructArrayLogItem<Translation3d>(name, Translation3d.struct,
                                    () -> (Translation3d[]) checkedValueSupplier.get(),
                                    logAnnotation.logLevel())),
                    entry(Twist2d.class,
                            () -> new StructLogItem<Twist2d>(name, Twist2d.struct,
                                    () -> (Twist2d) checkedValueSupplier.get(),
                                    logAnnotation.logLevel())),
                    entry(Twist2d[].class,
                            () -> new StructArrayLogItem<Twist2d>(name, Twist2d.struct,
                                    () -> (Twist2d[]) checkedValueSupplier.get(),
                                    logAnnotation.logLevel())),
                    entry(Twist3d.class,
                            () -> new StructLogItem<Twist3d>(name, Twist3d.struct,
                                    () -> (Twist3d) checkedValueSupplier.get(),
                                    logAnnotation.logLevel())),
                    entry(Twist3d[].class,
                            () -> new StructArrayLogItem<Twist3d>(name, Twist3d.struct,
                                    () -> (Twist3d[]) checkedValueSupplier.get(),
                                    logAnnotation.logLevel())),
                    entry(ArmFeedforward.class,
                            () -> new StructLogItem<ArmFeedforward>(name, ArmFeedforward.struct,
                                    () -> (ArmFeedforward) checkedValueSupplier.get(),
                                    logAnnotation.logLevel())),
                    entry(ElevatorFeedforward.class,
                            () -> new StructLogItem<ElevatorFeedforward>(name, ElevatorFeedforward.struct,
                                    () -> (ElevatorFeedforward) checkedValueSupplier.get(),
                                    logAnnotation.logLevel())),
                    entry(ChassisSpeeds.class,
                            () -> new StructLogItem<ChassisSpeeds>(name, ChassisSpeeds.struct,
                                    () -> (ChassisSpeeds) checkedValueSupplier.get(),
                                    logAnnotation.logLevel())),
                    entry(SwerveModulePosition.class,
                            () -> new StructLogItem<SwerveModulePosition>(name, SwerveModulePosition.struct,
                                    () -> (SwerveModulePosition) checkedValueSupplier.get(),
                                    logAnnotation.logLevel())),
                    entry(SwerveModuleState.class,
                            () -> new StructLogItem<SwerveModuleState>(name, SwerveModuleState.struct,
                                    () -> (SwerveModuleState) checkedValueSupplier.get(),
                                    logAnnotation.logLevel())),
                    entry(SwerveModulePosition[].class,
                            () -> new StructArrayLogItem<SwerveModulePosition>(name, SwerveModulePosition.struct,
                                    () -> (SwerveModulePosition[]) checkedValueSupplier.get(),
                                    logAnnotation.logLevel())),
                    entry(SwerveModuleState[].class,
                            () -> new StructArrayLogItem<SwerveModuleState>(name, SwerveModuleState.struct,
                                    () -> (SwerveModuleState[]) checkedValueSupplier.get(),
                                    logAnnotation.logLevel()))

            );

            Supplier<Logger> supp = classToLoggerMap.get(value.getClass());
            if (supp == null) {
                // if no match, use toString (helpful for logging enums)
                return Optional
                        .of(new StringLogItem(name, () -> checkedValueSupplier.get().toString(),
                                logAnnotation.logLevel()));
            } else {
                return Optional.of(supp.get());
            }
        } catch (Exception e) {
            e.printStackTrace();
            return Optional.empty();
        }
    }
}
