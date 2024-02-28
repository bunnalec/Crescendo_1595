package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utilities.Constants.HardwareID;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IndexerSubsystem extends SubsystemBase{

    TalonFX indexerMotor;
    //ColorSensorV3 colorSensor;
    //Color percievedColor;
    DigitalInput beamBreakSensor = new DigitalInput(0); //True when Unimposed
    boolean noteLoaded;

    public IndexerSubsystem() {
        indexerMotor = new TalonFX(HardwareID.indexerMotorCANId);
        indexerMotor.setNeutralMode(NeutralModeValue.Brake);
        //colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
        noteLoaded = false;
    }

    public void stopIndexer() {
        indexerMotor.set(0.0);
    }

    public void indexNoteIntake() {
        if (noteLoaded) {
            indexerMotor.set(0.2);
            return;
        }

        indexerMotor.set(0.0);
    }

    public void indexNoteOuttake() {
        indexerMotor.set(-0.2);
    }

    public void indexNoteLaunch() {
        indexerMotor.set(0.2);
    }

    //Used for Testing Purposes
    public void indexNoteIntakeDisregardLoading() {
        indexerMotor.set(0.2);
    }
    
    @Override
    public void periodic() {

        if (!beamBreakSensor.get()) {
            noteLoaded = true;
        }
        else {
            noteLoaded = false;
        }

        /*
        int proximity = colorSensor.getProximity();
        if (proximity > 1000) {
            noteLoaded = true;
        }
        else {
            noteLoaded = false;
        }
        */
    }

    public boolean isNoteLoaded() {
        return noteLoaded;
    }

    public void telemetry() {
        SmartDashboard.putNumber("Indexer Motor Velocity", indexerMotor.getVelocity().getValueAsDouble());
        //SmartDashboard.putNumber("Note Proximity", colorSensor.getProximity());
        SmartDashboard.putBoolean("Note Loaded", noteLoaded);
    }
}
