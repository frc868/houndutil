# HoundLog Docs

## `Loggable`

A `Loggable` is the base representation of anything that has a `init()` and a `run()` method and publishes itself to NetworkTables. `Logger`s and `LogGroup`s implement this.

## `Logger`

A `Logger` is an object that logs data under a single key (or subsystem). This is abstract so it can't be directly created, but the three classes below inherit it.

### `DeviceLogger`

A `DeviceLogger` is an object that will log values related to a single device. This can be a motor controller, gyroscope, FRC control system component, really anything that has data that you want to log about it. The general NetworkTables structure of this would be `HoundLog/subsystem/device_name/value_name`.

#### `LogItem`s

A `DeviceLogger` contains an array of `LogItem`s that define each of the device to be stored. `LogItem`s contain a key and a function that will return a value.

### `SingleItemLogger`

A `SingleItemLogger` is an object that will log a single item. This should typically be a single subsystem variable, like a PID setpoint or a specific state. The general NetworkTables structure of this would be `HoundLog/subsystem/value_name`.

### `SendableLogger`

A `SendableLogger` is basically the same as a `SingleItemLogger` except it sends a Sendable to NetworkTables as its "single item." The general NetworkTables structure of this would be `HoundLog/subsystem/value_name`.

## `LogGroup`

A LogGroup is basically a consolidation of a group of Loggers. This allows you to be able to add the entire LogGroup to the manager and not have to have a separate call for each Logger. A `LogGroup` implements a `Loggable`, and its `init()` and `run()` methods apply all the `Logger`s contained in the group.

## `LoggingManager`

All `Loggable`s should be added to a `LoggingManager` to allow values to be published to NetworkTables. This object is a singleton, and manages the entire logging system (as per the name). You can add either `Logger`s or `LogGroup`s to this, allowing for some additional flexibility with debug values.

## A note on subsystem names

Subsystem names are defined "all the way down," meaning that if a LogGroup has a subsystem name, so do its children. The current implementation is such that they can be defined at any point, and changes should propagate downwards (but not upwards). This may be changed in a future revision, but it allows one to define both a LogGroup and its Loggers with no subsystem, and only define the subsystem at the `LoggingManager` call level, reducing verbosity. See the examples below for more information.

## Examples

### Basic `SingleItemLogger`
```java
SingleItemLogger logger = new SingleItemLogger<Double>("Shooter", LogType.DOUBLE, "Speed Setpoint", shooter::getSetpoint);

LoggingManager.getInstance().addLogger(logger);
```
In this example, we're creating a new SingleItemLogger that is of type `Double` and will log under the "Shooter" subsystem. Also, we're adding it to the LoggingManager so that its values will actually be logged. `shooter::getSetpoint` is the value that will give you the data.

### Basic `DeviceLogger`
```java
DeviceLogger logger = new DeviceLogger<SparkMax>(l_primary, "Drivetrain", "Left Primary Motor Controller", new LogItem<?>[] {new LogItem<Double>(LogType.NUMBER, "Encoder Position", l_primary.getEncoder()::getPosition), LogLevel.INFO});

LoggingManager.getInstance().addLogger(logger);
```

Note that it is added the same way as before to the `LoggingManager`.

### `DeviceLogger` with `LogProfileBuilder`
