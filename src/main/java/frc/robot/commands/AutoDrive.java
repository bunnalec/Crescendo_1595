package frc.robot.commands;

import frc.lib.utilities.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class AutoDrive extends SequentialCommandGroup {
    public AutoDrive(DrivetrainSubsystem drivetrainSubsystem, Trajectory trajectory){

        //Below code was part of BaseTalonFXSwerve. If we go with PathWeaver, we do not need to generate our own trajectories like this.
        //If we go with a 3rd party option like PathPlanner, more modifications will be needed due to them using their own trajectory class.

        /* 
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                config);
*/
        var thetaController =
            new ProfiledPIDController(
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


        addCommands(
            new InstantCommand(() -> drivetrainSubsystem.setPose(trajectory.getInitialPose())),
            swerveControllerCommand
        );
    }
}