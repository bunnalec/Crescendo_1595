package frc.robot.commands.squences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Index;
import frc.robot.commands.Launch;
import frc.robot.commands.Index.IndexDirection;
import frc.robot.commands.Launch.LaunchDirection;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class LaunchSequence extends ParallelCommandGroup{
    LauncherSubsystem launcherSubsystem;
    IndexerSubsystem indexerSubsystem;
    LaunchDirection direction;

    public LaunchSequence(LauncherSubsystem launcherSubsystem, IndexerSubsystem indexerSubsystem, LaunchDirection direction) {
        this.launcherSubsystem = launcherSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.direction = direction;

        addCommands(
            new Launch(launcherSubsystem, direction),
            new SequentialCommandGroup(new WaitCommand(1), new Index(indexerSubsystem, IndexDirection.INTAKE_RECKLESS))
        );
    }
    
}
