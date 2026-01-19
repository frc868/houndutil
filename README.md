# houndutil

[![CI](https://github.com/frc868/houndutil/actions/workflows/main.yml/badge.svg)](https://github.com/frc868/houndutil/actions/workflows/main.yml)

A utility library for TechHOUNDS. Includes houndlog, houndauto, and houndlib.

# Mandatory git configuration for projects
```
git config --global submodules.recurse true
```

## How to add this to other projects

Open your robot project, and open a terminal in its folder. Run:

```sh
git submodule add https://github.com/frc868/houndutil
```

This will clone and add the houndutil submodule to your repository.

If cloning a project that already has houndutil, run the following command after cloning:
```sh
git submodule update --init --recursive
```

Now, edit `build.gradle` and `settings.gradle`.

In `build.gradle`, add:

```gradle
implementation project(":houndutil")
```

in the `dependencies` block. It should look like this:

```gradle
dependencies {
    implementation project(":houndutil")

    implementation wpi.java.deps.wpilib()
    implementation wpi.java.vendor.java()

    ...
}
```

In `settings.gradle`, add these lines at the end of the file:

```gradle
include ':houndutil'
rootProject.children[0].buildFileName = "submodule.gradle"
```

In `Main.java`, change your robot to a HoundRobot:

```java
RobotBase.startRobot(() -> new HoundRobot(() -> new RobotContainer()));
``` 

in the main method. It should look like this:

```java
public static void main(String... args) {
    RobotBase.startRobot(() -> new HoundRobot(() -> new RobotContainer()));
}
```

Remove `Robot.java` completely. Calling the `CommandScheduler` will be done by HoundRobot.

Now, import `houndutil` items by using:

```java
import com.techhounds.houndutil.houndlog.LoggingManager;
```

