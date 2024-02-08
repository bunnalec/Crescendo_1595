package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utilities.Constants.hardwareID;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IndexerSubsystem extends SubsystemBase{

    TalonFX indexerMotor;
    ColorSensorV3 colorSensor;
    Color percievedColor;
    boolean noteLoaded;

    public IndexerSubsystem() {
        indexerMotor = new TalonFX(hardwareID.indexerMotorCANId);
        indexerMotor.setNeutralMode(NeutralModeValue.Brake);
        colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
        noteLoaded = false;
    }

    //Default Command (Sets motor to zero power when no input is put in)
    public void stopIndexer() {
        indexerMotor.set(0.0);
    }

    public void indexNoteIntake() {
        if (noteLoaded) {
            indexerMotor.set(0.0);
            return;
        }

        indexerMotor.set(1.0);
    }

    public void indexNoteOuttake() {
        indexerMotor.set(-1.0);
    }

    public void indexNoteShooter() {
        indexerMotor.set(1.0);
    }
    
    @Override
    public void periodic() {
        int proximity = colorSensor.getProximity();

        if (proximity > 1000) {
            noteLoaded = true;
        }
        else {
            noteLoaded = false;
        }
    }

    public boolean isNoteLoaded() {
        return noteLoaded;
    }

    public void telemetry() {
        SmartDashboard.putNumber("Indexer Motor Velocity", indexerMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Note Proximity", colorSensor.getProximity());
        SmartDashboard.putBoolean("Note Loaded", noteLoaded);
    }
}
