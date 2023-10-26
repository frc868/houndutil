package com.techhounds.houndutil.houndauto.trajectoryloader;

import java.io.IOException;
import java.nio.file.DirectoryStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Function;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.techhounds.houndutil.houndauto.AutoPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

public class TrajectoryLoader {
    private static HashMap<String, AutoPath> autoPaths = new HashMap<String, AutoPath>();
    private static HashMap<String, TrajectorySettings> trajectorySettingsMap = new HashMap<String, TrajectorySettings>();

    public static void loadAutoPaths() {
        try {
            DirectoryStream.Filter<Path> filter = new DirectoryStream.Filter<Path>() {
                public boolean accept(Path filename) throws IOException {
                    return filename.getFileName().toString().endsWith(".path");
                }
            };
            DirectoryStream<Path> trajectoryFiles = Files
                    .newDirectoryStream(Filesystem.getDeployDirectory().toPath().resolve("pathplanner"), filter);

            Function<Path, String> stripPath = (path) -> {
                String filename = path.getFileName().toString();
                return filename.substring(0, filename.length() - 5);
            };
            // trajectoryFiles.forEach(path -> {
            //     String trajName = stripPath.apply(path);
            //     TrajectorySettings settings = trajectorySettingsMap.get(trajName);
            //     if (settings == null) {
            //         settings = new TrajectorySettings(trajName); // to set default maxV and maxA
            //     }

            //     ArrayList<PathConstraints> constraints = settings.getConstraints();
            //     ArrayList<PathPlannerTrajectory> trajectories;
            //     PathConstraints firstConstraint = constraints.get(0);
            //     if (constraints.size() == 1) {
            //         trajectories = new ArrayList<PathPlannerTrajectory>(
            //                 PathPlannerPath.loadPathGroup(trajName, firstConstraint));
            //     } else {
            //         PathConstraints[] otherConstraints = new PathConstraints[constraints.size() - 1];
            //         constraints.subList(1, constraints.size()).toArray(otherConstraints);

            //         trajectories = new ArrayList<PathPlannerTrajectory>(
            //                 PathPlanner.loadPathGroup(trajName, firstConstraint, otherConstraints));
            //     }

            //     autoPaths.put(trajName, new AutoPath(trajName, trajectories));
            // }); // TODO
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectories", ex.getStackTrace());
        }
    }

    public static AutoPath getAutoPath(String name) {
        return autoPaths.get(name);
    }

    // /**
    // * Creates an {@code ArrayList} of the specified {@code AutoPath}s. Use when
    // * creating an {@code AutoTrajectoryCommand}.
    // *
    // * @param names
    // * @return
    // */
    // public static ArrayList<PPAutoPath> getAutoPaths(String... names) {
    // ArrayList<PPAutoPath> arr = new ArrayList<PPAutoPath>();
    // for (String name : names) {
    // PPAutoPath autoPath = autoPaths.get(name);
    // if (autoPath == null) {
    // throw new IllegalArgumentException(name + " does not exist");
    // }
    // arr.add(autoPath);
    // }
    // return arr;
    // }

    public static void addSettings(TrajectorySettings... settings) {
        for (TrajectorySettings setting : settings) {
            trajectorySettingsMap.put(setting.name, setting);
        }
    }
}
