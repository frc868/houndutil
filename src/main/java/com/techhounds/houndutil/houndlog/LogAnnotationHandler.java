package com.techhounds.houndutil.houndlog;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleConsumer;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.DigitalInput;

import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LogProfile;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;
import com.techhounds.houndutil.houndlog.annotations.SendableLog;
import com.techhounds.houndutil.houndlog.annotations.Tunable;
import com.techhounds.houndutil.houndlog.loggers.BooleanArrayLogItem;
import com.techhounds.houndutil.houndlog.loggers.BooleanLogItem;
import com.techhounds.houndutil.houndlog.loggers.DoubleArrayLogItem;
import com.techhounds.houndutil.houndlog.loggers.DoubleLogItem;
import com.techhounds.houndutil.houndlog.loggers.FloatArrayLogItem;
import com.techhounds.houndutil.houndlog.loggers.FloatLogItem;
import com.techhounds.houndutil.houndlog.loggers.IntegerArrayLogItem;
import com.techhounds.houndutil.houndlog.loggers.IntegerLogItem;
import com.techhounds.houndutil.houndlog.loggers.LogGroup;
import com.techhounds.houndutil.houndlog.loggers.LogItem;
import com.techhounds.houndutil.houndlog.loggers.Loggable;
import com.techhounds.houndutil.houndlog.loggers.SendableLogItem;
import com.techhounds.houndutil.houndlog.loggers.StringArrayLogItem;
import com.techhounds.houndutil.houndlog.loggers.StringLogItem;
import com.techhounds.houndutil.houndlog.loggers.StructArrayLogItem;
import com.techhounds.houndutil.houndlog.loggers.StructLogItem;
import com.techhounds.houndutil.houndlog.loggers.TunableBoolean;
import com.techhounds.houndutil.houndlog.loggers.TunableDouble;

import static java.util.Map.entry;

/**
 * Handles all objects that have been annotated with {@code @Log}, adding them
 * to the {@link LoggingManager}.
 * 
 * Recurses through all objects that have been annotated with
 * {@code @LoggedObject}, and iterates through all fields and methods checking
 * for annotations.
 */
public class LogAnnotationHandler {
    /**
     * Generates loggers for a given object that has been marked with
     * {@code @LoggedObject}.
     * 
     * @param loggedObject the object to handle
     */
    protected static void handleLoggedObject(Object loggedObject) {
        handleLoggedObject(loggedObject, "", new ArrayList<String>());
    }

    /**
     * Generates loggers for a given object that has been marked with
     * {@code @LoggedObject}.
     * 
     * @param loggedObject the object to handle
     * @param name         the name of the object
     * @param subkeys      a list of logged subclasses containing this object
     */
    protected static void handleLoggedObject(Object loggedObject, String name, ArrayList<String> subkeys) {
        handleLoggedObjectImpl(
                loggedObject,
                loggedObject.getClass().getDeclaredFields(),
                loggedObject.getClass().getDeclaredMethods(),
                name, subkeys);
    }

    /**
     * Generates loggers for a class that has been annotated with
     * {@code LoggedObject}. Used for classes with <b>static</b> fields and methods.
     * 
     * @param loggedClass the class to handle
     * @param name        the name of the object
     * @param subkeys     a list of logged subclasses containing this object
     */
    protected static void handleLoggedClass(Class<?> loggedClass, String name, ArrayList<String> subkeys) {
        handleLoggedObjectImpl(
                null,
                loggedClass.getDeclaredFields(),
                loggedClass.getDeclaredMethods(),
                name, subkeys);
    }

    /**
     * Implementation for handling a logged object. Checks for {@code @Log},
     * {@code @Tunable}, and {@code @SendableLog} annotations on fields and methods
     * in the object, and recursively calls
     * {@link LogAnnotationHandler#handleLoggedObject} for any discovered
     * {@code @LoggedObject} fields.
     * 
     * @param loggedObject the object to handle
     * @param fields       the fields to search
     * @param methods      the methods to search
     * @param name         the name of the objects
     * @param subkeys      a list of logged subclasses containing this object
     */
    private static void handleLoggedObjectImpl(Object loggedObject, Field[] fields, Method[] methods, String name,
            ArrayList<String> subkeys) {
        ArrayList<Loggable> loggers = new ArrayList<Loggable>();
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

                Optional<Loggable> optLogger = getLoggerForValue(valueSupplier, subLogAnnotation, varName);
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
                        Optional<Loggable> optLogger = getLoggerForValue(valueSupplier, subLogAnnotation, varName);
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
                String varName = subSendableLogAnnotation.name().equals("") ? field.getName()
                        : subSendableLogAnnotation.name();
                nameComponents.add(varName);
                String formattedName = String.join("/", nameComponents);

                try {
                    Sendable sendable = (Sendable) field.get(loggedObject);
                    loggers.add(new SendableLogItem(formattedName, sendable));
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }

        }
        if (name != null)
            subkeys.add(name);
        LoggingManager.getInstance().addGroup(
                new LogGroup(String.join("/", subkeys), loggers.toArray(new Loggable[loggers.size()])));
    }

    /**
     * Generates the name for a logged field given the field's name and the desired
     * subkeys.
     * 
     * @param logAnnotation the log annotation
     * @param varName       the name of the variable
     * @return the fully qualified name of the object
     */
    public static String getName(Log logAnnotation, String varName) {
        ArrayList<String> nameComponents = new ArrayList<String>();
        nameComponents.addAll(Arrays.asList(logAnnotation.groups()));
        nameComponents.add(varName);
        return String.join("/", nameComponents);
    }

    /**
     * Generates the correct logger (group or single item) for a given value, if
     * possible.
     * 
     * @param valueSupplier supplier for the value of the object
     * @param logAnnotation the log annotation attached to the variable
     * @param varName       the name of the variable
     * @return a logger, if one exists for the given object type
     */
    @SuppressWarnings("unchecked")
    private static Optional<Loggable> getLoggerForValue(Supplier<Object> valueSupplier, Log logAnnotation,
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

            // for primitives, use the appropriate logger
            Map<Class<?>, Supplier<Loggable>> classToLoggerMap = Map.ofEntries(
                    entry(boolean[].class,
                            () -> new BooleanArrayLogItem(name,
                                    () -> (boolean[]) checkedValueSupplier.get(), logAnnotation.logType())),
                    entry(Boolean.class,
                            () -> new BooleanLogItem(name,
                                    () -> (boolean) checkedValueSupplier.get(), logAnnotation.logType())),
                    entry(double[].class,
                            () -> new DoubleArrayLogItem(name,
                                    () -> (double[]) checkedValueSupplier.get(), logAnnotation.logType())),
                    entry(Double.class,
                            () -> new DoubleLogItem(name,
                                    () -> (double) checkedValueSupplier.get(), logAnnotation.logType())),
                    entry(float[].class,
                            () -> new FloatArrayLogItem(name,
                                    () -> (float[]) checkedValueSupplier.get(), logAnnotation.logType())),
                    entry(Float.class,
                            () -> new FloatLogItem(name,
                                    () -> (float) checkedValueSupplier.get(), logAnnotation.logType())),
                    entry(int[].class,
                            () -> new IntegerArrayLogItem(name,
                                    () -> (int[]) checkedValueSupplier.get(), logAnnotation.logType())),
                    entry(Integer.class,
                            () -> new IntegerLogItem(name,
                                    () -> (int) checkedValueSupplier.get(), logAnnotation.logType())),
                    entry(String[].class,
                            () -> new StringArrayLogItem(name,
                                    () -> (String[]) checkedValueSupplier.get(), logAnnotation.logType())),
                    entry(String.class,
                            () -> new StringLogItem(name,
                                    () -> (String) checkedValueSupplier.get(), logAnnotation.logType())),
                    entry(DigitalInput.class,
                            () -> new BooleanLogItem(name,
                                    () -> ((DigitalInput) checkedValueSupplier.get()).get())));

            Supplier<Loggable> supp = classToLoggerMap.get(value.getClass());
            if (supp != null) {
                return Optional.of(supp.get());
            }

            // if something is covered by a profile, use it
            Map<Class<?>, Function<Object, LogItem<?>[]>> profiles = LoggingManager.getInstance().getProfiles();
            Function<Object, LogItem<?>[]> profile = profiles.get(value.getClass());
            if (profile != null) {
                return Optional.of(new LogGroup(name,
                        profile.apply(checkedValueSupplier.get())));
            }

            // if a struct, use the struct logger
            if (value instanceof StructSerializable) {
                try {
                    Class<?> clazz = value.getClass();
                    Field structField = clazz.getField("struct");

                    if (Modifier.isStatic(structField.getModifiers())
                            && Struct.class.isAssignableFrom(structField.getType())) {
                        Struct<Object> structValue = (Struct<Object>) structField.get(null);

                        return Optional.of(new StructLogItem<Object>(name, structValue,
                                () -> checkedValueSupplier.get(),
                                logAnnotation.logType()));
                    }
                } catch (NoSuchFieldException | IllegalAccessException e) {
                    // Handle exceptions, such as the field not being accessible
                    e.printStackTrace();
                }
                // if an array of structs, use the struct array logger
            } else if (value instanceof Object[]) {
                Object[] arr = (Object[]) value;
                if (arr.length > 0 && arr[0] instanceof StructSerializable) {
                    try {
                        Class<?> clazz = arr[0].getClass();
                        Field structField = clazz.getField("struct");

                        if (Modifier.isStatic(structField.getModifiers())
                                && Struct.class.isAssignableFrom(structField.getType())) {
                            Struct<Object> structValue = (Struct<Object>) structField.get(null);

                            return Optional.of(new StructArrayLogItem<Object>(name, structValue,
                                    () -> (Object[]) checkedValueSupplier.get(),
                                    logAnnotation.logType()));
                        }
                    } catch (NoSuchFieldException | IllegalAccessException e) {
                        // Handle exceptions, such as the field not being accessible
                        e.printStackTrace();
                    }
                }
            }

            // if no match, use toString (valid for logging enums, for example)
            return Optional.of(new StringLogItem(name, () -> checkedValueSupplier.get().toString(),
                    logAnnotation.logType()));
        } catch (Exception e) {
            e.printStackTrace();
            return Optional.empty();
        }
    }

    /**
     * Generates map of log profiles for a given annotated class (containing methods
     * annotated with {@code @LogProfile}, each taking in an object and outputting
     * an array of LogItems).
     * 
     * @param clazz    the class to search for {@code @LogProfile} annotations
     * @param profiles the map to add the profiles to
     */
    protected static void handleLogProfile(Class<?> clazz, Map<Class<?>, Function<Object, LogItem<?>[]>> profiles) {
        for (Method method : clazz.getMethods()) {
            LogProfile profileAnnotation = method.getAnnotation(LogProfile.class);
            if (profileAnnotation != null) {
                Class<?> forClass = profileAnnotation.value();
                method.setAccessible(true);

                profiles.put(forClass, (o) -> {
                    try {
                        return (LogItem<?>[]) method.invoke(null, o);
                    } catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e) {
                        e.printStackTrace();
                        return new LogItem[0];
                    }
                });
            }
        }
    }
}
