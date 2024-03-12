package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.utilities.LimelightHelpers;
import frc.lib.utilities.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Align extends Command {

    public final int speakerTagId;
    public double wantedX = 0;
    public double wantedY = 0;
    public double wantedDirection = 0;
    public DrivetrainSubsystem drivetrainSubsystem;


    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry tolerance;

    NetworkTableEntry currentErrorX;
    NetworkTableEntry currentErrorY;
    NetworkTableEntry tagVisible;

    public Align(int speakerTagId, double wantedX, double wantedY, double wantedDirection, DrivetrainSubsystem drivetrainSubsystem) {
        this.speakerTagId = speakerTagId;
        this.wantedX = wantedX;
        this.wantedY = wantedY;
        this.wantedDirection = wantedDirection;
        this.drivetrainSubsystem = drivetrainSubsystem;

        NetworkTable table = NetworkTableInstance.getDefault().getTable("alignment_pid");
        tx = table.getEntry("x_factor");
        ty = table.getEntry("y_factor");
        tolerance = table.getEntry("tolerance");

        currentErrorX = table.getEntry("error_x");
        currentErrorY = table.getEntry("error_y");
        tagVisible = table.getEntry("visible");

        tx.setDouble(10.0);
        ty.setDouble(10.0);
        tolerance.setDouble(0.1);
    }

     /** The initial subroutine of a command. Called once when the command is initially scheduled. */
  public void initialize() {}

  /** The main body of a command. Called repeatedly while the command is scheduled. */
  public void execute() {
    // has the camera always check the april tag id, tx, and ty
    //   Which ID? What are the right values for tx and ty?
    
    // take the tx and ty and then devide them by themselves so that at first the power is at one and as the robot
    // gets closer, the power goes down with it stopping when the power gets low enough.
 
    Translation2d error = findTargetsError();

    if (error == null) {
        // What to do when the april tag isn't visible?
        return;
    }

    double translationValue = error.getY() * ty.getDouble(0.0);
    double strafeValue = error.getX() * tx.getDouble(0.0);
    
    drivetrainSubsystem.drive(
      new Translation2d(translationValue, strafeValue),
      0,
      false,
      false);
  }

  Translation2d findTargetsError() {
    LimelightTarget_Fiducial[] aprilTags = LimelightHelpers.getLatestResults("limelight").targetingResults.targets_Fiducials;

    for (LimelightTarget_Fiducial aprilTag: aprilTags) {
        if (aprilTag.fiducialID == speakerTagId) {
            double xError = MathUtil.applyDeadband(aprilTag.tx - wantedX, tolerance.getDouble(0.1));
            double yError = MathUtil.applyDeadband(aprilTag.ty - wantedY, tolerance.getDouble(0.1));
            currentErrorX.setDouble(xError);
            currentErrorY.setDouble(yError);
            tagVisible.setBoolean(true);
            return new Translation2d(xError, yError);
        }
    }

    tagVisible.setBoolean(false);
    return null;
  }

  /**
   * The action to take when the command ends. Called when either the command finishes normally, or
   * when it interrupted/canceled.
   *
   * <p>Do not schedule commands here that share requirements with this command. Use {@link
   * #andThen(Command...)} instead.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  public void end(boolean interrupted) {
    // turns off all motors
  }

  /**
   * Whether the command has finished. Once a command finishes, the scheduler will call its end()
   * method and un-schedule it.
   *
   * @return whether the command has finished.
   */
  public boolean isFinished() {
    Translation2d error = findTargetsError();

    if (error == null) {
        // Keep trying?
        return false;
    }
    if (error.getX() == 0 && error.getY() == 0) {
        return true;
    } else {
        return false;
    }
    // check if aligned with correct april tag
  }
    
}
