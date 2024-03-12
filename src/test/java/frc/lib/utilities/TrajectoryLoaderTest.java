package frc.lib.utilities;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class TrajectoryLoaderTest {

    @Test
    public void parseJson() throws Exception {

        String testPath = """
                {
                    "start": {
                        "translation": {
                          "x": 0.5,
                          "y": 1.0
                        },
                        "rotation": {
                          "radians": 0.0
                        }
                    },
                    "waypoints": [
                        { "x": 1.0, "y": 1.0 },
                        { "x": 1.0, "y": 2.0 }
                    ],
                    "end": {
                        "translation": {
                          "x": 1.5,
                          "y": 2.0
                        },
                        "rotation": {
                          "radians": 0.0
                        }
                    }
                }
                """;

        NetworkTable sgsProps = NetworkTableInstance.getDefault().getTable("sgs");
        TrajectoryLoader loader = new TrajectoryLoader();

        assertNull(loader.loadFromNetworkTable());
        sgsProps.getEntry("path").setString(testPath);

        assertNull(loader.loadFromNetworkTable());
        sgsProps.getEntry("path_speed").setDouble(1.0);

        assertNull(loader.loadFromNetworkTable());
        sgsProps.getEntry("path_accel").setDouble(1.0);

        Trajectory traj = loader.loadFromNetworkTable();
        assertNotNull(traj);

        assertEquals(traj.getInitialPose().getX(), 0.5);

    }

    @Test
    public void testResources() {
        TrajectoryLoader loader = new TrajectoryLoader(0.1, 0.1);

        assertNull(loader.loadFromResource("/nonexistent"));

        // The leading slash means the resources is at the root of the classpath
        // and not relative to the TrajectoryLoader's package.
        // "FirstDrive.json" = "/frc/lib/utilities/FirstDrive.json"
        // "/FirstDrive.json" = "/FirstDrive.json"
        assertNotNull(loader.loadFromResource("/FirstDrive.json"));
    }

}