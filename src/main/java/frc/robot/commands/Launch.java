// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class Launch extends Command {

  public enum LaunchDirection {
    LOW,
    HIGH,
    DROP //If Amp Requires it
  }

  private final LauncherSubsystem launcherSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final LaunchDirection direction;

  public Launch(LauncherSubsystem launcherSubsystem, IndexerSubsystem indexerSubsystem, LaunchDirection direction) {
    this.launcherSubsystem = launcherSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    addRequirements(launcherSubsystem, indexerSubsystem);

    this.direction = direction;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch (direction) {
      case LOW:
        launcherSubsystem.spinLowerSpinners();
        break;
      case HIGH:
        launcherSubsystem.spinUpperSpinners();
        break;
      case DROP:
        launcherSubsystem.spinLowerSpinners(0.1);
        break;
      default: break;
    }
    indexerSubsystem.indexNoteLaunch();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launcherSubsystem.stopSpinners();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

