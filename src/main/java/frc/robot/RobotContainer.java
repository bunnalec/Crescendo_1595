// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.utilities.Constants;
import frc.lib.utilities.Constants.*;

import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();

  private final Joystick driver = new Joystick(OperatorConstants.driverControllerPort);

  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

  //Drive System Identification
  private final JoystickButton quasiForward = new JoystickButton(driver, XboxController.Button.kBack.value);
  private final JoystickButton quasiBackward = new JoystickButton(driver, XboxController.Button.kStart.value);

  private final JoystickButton dynamicForward = new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton dynamicBackward = new JoystickButton(driver, XboxController.Button.kB.value);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    drivetrainSubsystem.setDefaultCommand(
      new TeleopDrive(
          drivetrainSubsystem,
          () -> -driver.getRawAxis(translationAxis),
          () -> -driver.getRawAxis(strafeAxis),
          () -> driver.getRawAxis(rotationAxis),
          () -> robotCentric.getAsBoolean()
        )
    );

    // Configure button bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    zeroGyro.onTrue(new InstantCommand(() -> drivetrainSubsystem.zeroHeading()));

    if (Constants.SystemIdentificationToggles.driveSystemIdentification) {
      quasiForward.whileTrue(drivetrainSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      quasiBackward.whileTrue(drivetrainSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      dynamicForward.whileTrue(drivetrainSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
      dynamicBackward.whileTrue(drivetrainSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

   /*
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
  */
}
