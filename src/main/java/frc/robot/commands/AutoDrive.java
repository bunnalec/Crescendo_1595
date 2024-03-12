package frc.robot.commands;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.lib.utilities.Constants;
import frc.lib.utilities.TrajectoryLoader;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

import java.util.List;

public class AutoDrive extends SequentialCommandGroup {

    public AutoDrive(DrivetrainSubsystem drivetrainSubsystem, LauncherSubsystem launcher, IndexerSubsystem indexer){

        TrajectoryLoader loader = new TrajectoryLoader();

        Trajectory trajectory = loader.loadFromNetworkTable();

        var thetaController = new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                trajectory,
                drivetrainSubsystem::getPose,
                Constants.SwerveConstants.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                drivetrainSubsystem::setModuleStates,
                drivetrainSubsystem);

        addCommands(new Align(6, 0, 0, 0, drivetrainSubsystem));
        // boolean isBlue = false;
        // if (isBlue) {
        //     addCommands(
        //         new Move(),
        //         new Align(),
        //         new Launch(launcher,  indexer, LaunchDirection.LOW),
        //         new Index(indexer, IndexDirection.INTAKE)
        //         new Move(),
        //         new Align(),
        //         new Launch(launcher,  indexer, LaunchDirection.LOW)
        //     );
        // } else {
        //     addCommands(
        //         new Move(),
        //         new Align(),
        //         new Launch(launcher,  indexer, LaunchDirection.LOW),
        //         new Index(indexer, IndexDirection.INTAKE)
        //         new Move(),
        //         new Align(),
        //         new Launch(launcher,  indexer, LaunchDirection.LOW)
        //     );

        // }
    }
}