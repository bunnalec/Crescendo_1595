// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.utilities.Constants;
import frc.lib.utilities.Constants.OperatorConstants;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.Index;
import frc.robot.commands.Index.IndexDirection;
import frc.robot.commands.Launch;
import frc.robot.commands.Launch.LaunchDirection;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class RobotContainer {
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  private final LauncherSubsystem launcherSubsystem = new LauncherSubsystem();

  private final Joystick driver = new Joystick(OperatorConstants.driverControllerPort);
  private final Joystick operator = new Joystick(OperatorConstants.operatorControllerPort);

  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kStart.value);
  private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

  private final JoystickButton launchDrop = new JoystickButton(operator, XboxController.Button.kX.value);
  private final JoystickButton launchLow = new JoystickButton(operator, XboxController.Button.kA.value);
  private final JoystickButton launchHigh = new JoystickButton(operator, XboxController.Button.kY.value);

  private final JoystickButton indexerIntake = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
  private final JoystickButton indexerOuttake = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    drivetrainSubsystem.setDefaultCommand(
      new TeleopDrive(
          drivetrainSubsystem,
          () -> -driver.getRawAxis(translationAxis),
          () -> -driver.getRawAxis(strafeAxis),
          () -> -driver.getRawAxis(rotationAxis),
          () -> robotCentric.getAsBoolean()
        )
    );

    // Configure button bindings
    configureBindings();
  }

  private void configureBindings() {

    //The keybinds and commands for system identification only load if the mode is enabled in constants (for programming purposes).
    if (Constants.SystemIdentificationToggles.systemIdentification) {
      JoystickButton driveQuasiForward = new JoystickButton(driver, XboxController.Button.kBack.value);
      JoystickButton driveQuasiBackward = new JoystickButton(driver, XboxController.Button.kStart.value);
      JoystickButton driveDynamicForward = new JoystickButton(driver, XboxController.Button.kX.value);
      JoystickButton driveDynamicBackward = new JoystickButton(driver, XboxController.Button.kB.value);


      driveQuasiForward.whileTrue(drivetrainSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      driveQuasiBackward.whileTrue(drivetrainSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      driveDynamicForward.whileTrue(drivetrainSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
      driveDynamicBackward.whileTrue(drivetrainSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
      
      //No Other Keybinds will Load in System Identification Mode, for no other keybinds will be assigned to actions.
      return;
    }

    //Keybinds for... actually driving the robot in TeleOP.
    zeroGyro.onTrue(new InstantCommand(() -> drivetrainSubsystem.zeroHeading()));

    launchDrop.whileTrue(new Launch(launcherSubsystem, indexerSubsystem, LaunchDirection.DROP));
    launchLow.whileTrue(new Launch(launcherSubsystem, indexerSubsystem, LaunchDirection.LOW));
    launchHigh.whileTrue(new Launch(launcherSubsystem, indexerSubsystem, LaunchDirection.HIGH));

    indexerIntake.whileTrue(new Index(indexerSubsystem, IndexDirection.INTAKE_RECKLESS)); //Reckless until sensor gets attatched.
    indexerOuttake.whileTrue(new Index(indexerSubsystem, IndexDirection.OUTTAKE));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new AutoDrive(drivetrainSubsystem, launcherSubsystem, indexerSubsystem);
  }
}
