// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utilities.Constants.SwerveConstants;
import frc.lib.utilities.swerve.SwerveModule;

public class DrivetrainSubsystem extends SubsystemBase {
  public SwerveDriveOdometry swerveDriveOdometry;
  public SwerveModule[] swerveModules;
  public AHRS navx;

  public DrivetrainSubsystem() {
    //Initializes Gyroscope (Measures Yaw/Rotation Angle), swerve modules, and swerve odometry.
    navx = new AHRS(SPI.Port.kMXP, (byte) 200);
    navx.reset();

    swerveModules = new SwerveModule[] {
      new SwerveModule(0, SwerveConstants.Mod0.constants),
      new SwerveModule(1, SwerveConstants.Mod1.constants),
      new SwerveModule(2, SwerveConstants.Mod2.constants),
      new SwerveModule(3, SwerveConstants.Mod3.constants)
    };
    
    swerveDriveOdometry = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, getGyroYaw(), getModulePositions());
  }

  public void drive(Translation2d translation, double anglularVelocity, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
      SwerveConstants.swerveKinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                anglularVelocity,
                getHeading()
              )
              : new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                anglularVelocity)
              );
      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

      for (SwerveModule module : swerveModules) {
        module.setDesiredState(swerveModuleStates[module.moduleNumber], isOpenLoop);
      }
  }

  // Used by SwerveControllerCommand in Auto
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxSpeed);

    for (SwerveModule module : swerveModules) {
      module.setDesiredState(desiredStates[module.moduleNumber], false);
    }
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];

    for(SwerveModule module : swerveModules) {
      states[module.moduleNumber] = module.getState();
    }

    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for (SwerveModule module : swerveModules) {
      positions[module.moduleNumber] = module.getPosition();
    }

    return positions;
  }

  public Pose2d getPose() {
    return swerveDriveOdometry.getPoseMeters();
  }

  public void setPose(Pose2d pose) {
    swerveDriveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public void setHeading(Rotation2d heading) {
    swerveDriveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
  }

  public void zeroHeading() {
    swerveDriveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
  }

  public Rotation2d getGyroYaw() {
    return Rotation2d.fromDegrees(-navx.getAngle()); //Negated because NavX CW Positive while ChassisSpeeds requires CCW Positive
    /*return Rotation2d.fromDegrees(360 - navx.getYaw());*/ //Old, non-continous method of obtaining yaw. Above aligns more with behavior of Pigeon 2.
  }

  public void resetModulesToAbsolute() {
    for(SwerveModule module : swerveModules) {
      module.resetToAbsolute();
    }
  }

  private void telemetry() {
    for(SwerveModule module : swerveModules){
            SmartDashboard.putNumber("Module " + module.moduleNumber + " CANcoder", module.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Module " + module.moduleNumber + " Angle", module.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Module " + module.moduleNumber + " Velocity", module.getState().speedMetersPerSecond);
          }     
    SmartDashboard.putNumber("Heading", getHeading().getDegrees());
  }

  @Override
  public void periodic() {
    swerveDriveOdometry.update(getGyroYaw(), getModulePositions());
    telemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
