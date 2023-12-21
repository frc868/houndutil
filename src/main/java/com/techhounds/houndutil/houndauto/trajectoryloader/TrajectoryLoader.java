package com.techhounds.houndutil.houndauto.trajectoryloader;

import java.io.IOException;
import java.nio.file.DirectoryStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.function.Function;

import com.pathplanner.lib.path.PathPlannerPath;
import com.techhounds.houndutil.houndauto.AutoPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

public class TrajectoryLoader {
    private static HashMap<String, AutoPath> autoPaths = new HashMap<String, AutoPath>();

    public static void loadAutoPaths() {
        try {
            DirectoryStream.Filter<Path> filter = new DirectoryStream.Filter<Path>() {
                public boolean accept(Path filename) throws IOException {
                    return filename.getFileName().toString().endsWith(".path");
                }
            };
            DirectoryStream<Path> trajectoryFiles = Files
                    .newDirectoryStream(Filesystem.getDeployDirectory().toPath().resolve("pathplanner"), filter);

            Function<Path, String> stripPath = (filePath) -> {
                String filename = filePath.getFileName().toString();
                return filename.substring(0, filename.length() - 5);
            };

            trajectoryFiles.forEach(filePath -> {
                String trajName = stripPath.apply(filePath);
                PathPlannerPath path = PathPlannerPath.fromPathFile(trajName);

                autoPaths.put(trajName, new AutoPath(trajName, path));
            });
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectories", ex.getStackTrace());
        }
    }

    public static AutoPath getAutoPath(String name) {
        System.out.println(autoPaths.size());
        return autoPaths.get(name);
    }
}
