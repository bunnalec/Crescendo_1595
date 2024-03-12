package frc.lib.utilities;

import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.Collections;
import java.util.List;

public class TrajectoryLoader {

    public static class SgsTrajectory {
        public Pose2d start = null;
        public List<Translation2d> waypoints = Collections.emptyList();

        public Pose2d end = null;

    }

    public static double defaultSpeed = 0.0;
    public static double defaultAccel = 0.0;
    private static final ObjectMapper mapper = new ObjectMapper()
            .enable(JsonParser.Feature.ALLOW_COMMENTS);

    private NetworkTable sgsProps = NetworkTableInstance.getDefault().getTable("sgs");
    private NetworkTableEntry ntPath = sgsProps.getEntry("path");
    private NetworkTableEntry ntSpeed = sgsProps.getEntry("path_speed");
    private NetworkTableEntry ntAccel = sgsProps.getEntry("path_accel");

    private final double speed;
    private final double accel;

    public TrajectoryLoader() {
        this(defaultSpeed, defaultAccel);
    }

    public TrajectoryLoader(double speed, double accel) {
        this.speed = speed;
        this.accel = accel;
        ntPath.setString("{}");
        ntSpeed.setDouble(speed);
        ntAccel.setDouble(accel);
    }

    public Trajectory loadFromNetworkTable() {
        String json = ntPath.getString("{}");
        double speed = ntSpeed.getDouble(0.0);
        double accel = ntAccel.getDouble(0.0);
        return createTrajectory(json, speed, accel);
    }

    public Trajectory loadFromResource(String name) {

        final String json;
        try (InputStream resource = getClass().getResourceAsStream(name)) {
            if (resource == null) {
                DriverStation.reportError("Unable to find path resource [" + name + "]", false);
                return null;
            }

            json = new String(resource.readAllBytes(), StandardCharsets.UTF_8);
        } catch (Exception e) {
            DriverStation.reportError("Unable to read path resource [" + name + "]: " + e.getMessage(), false);
            return null;
        }

        return createTrajectory(json, this.speed, this.accel);
    }

    private static Trajectory createTrajectory(String json, double speed, double accel) {
        if (json.equals("{}")) {
            DriverStation.reportWarning("No path configured in NT sgs.path", false);
            return null;
        }

        if (speed == 0.0) {
            DriverStation.reportWarning("NT sgs.path_speed is 0. Can't move", false);
            return null;
        }

        if (accel == 0.0) {
            DriverStation.reportWarning("NT sgs.path_accel is 0. Can't accelerate", false);
            return null;
        }

        try {
            SgsTrajectory path = mapper.readValue(json, SgsTrajectory.class);
            if (path.start == null || path.end == null) {
                DriverStation.reportWarning("Trajectories must have a start and an end", false);
                return null;
            }

            TrajectoryConfig config = new TrajectoryConfig(speed, accel)
                    .setKinematics(Constants.SwerveConstants.swerveKinematics);

            return TrajectoryGenerator.generateTrajectory(
                    path.start,
                    path.waypoints,
                    path.end,
                    config);

        } catch (Exception e) {
            DriverStation.reportWarning("Loading trajectory json failed: " + e.getMessage(), false);
            return null;
        }
    }
}
