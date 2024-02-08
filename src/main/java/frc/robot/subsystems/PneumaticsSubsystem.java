package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utilities.Constants.hardwareID;

public class PneumaticsSubsystem extends SubsystemBase{

    public DoubleSolenoid leftClimber, rightClimber, noteAimer;
    boolean allSolenoidsDisabled = false;
    Compressor compressor;


    public PneumaticsSubsystem() {
        leftClimber = new DoubleSolenoid(PneumaticsModuleType.REVPH, hardwareID.leftClimberForwardChannel, hardwareID.leftClimberReverseChannel);
        rightClimber = new DoubleSolenoid(PneumaticsModuleType.REVPH, hardwareID.rightClimberForwardChannel, hardwareID.rightClimberReverseChannel);
        noteAimer = new DoubleSolenoid(PneumaticsModuleType.REVPH, hardwareID.noteAimerForwardChannel, hardwareID.noteAimerReverseChannel);


        compressor = new Compressor(PneumaticsModuleType.REVPH);
        compressor.enableDigital();

        setAllSolenoidsToReverse();
    }

    
    //Individual Solenoid Controls (Take Solenoid as Input)
    public void setSolenoidToReverse(DoubleSolenoid solenoid) {
        solenoid.set(Value.kReverse);
    }
    
    public void setSolenoidToForward(DoubleSolenoid solenoid) {
        solenoid.set(Value.kForward);
    }

    public void turnSolenoidOff(DoubleSolenoid solenoid) {
        solenoid.set(Value.kOff);
    }

    public void toggleSolenoid(DoubleSolenoid solenoid) {
        solenoid.toggle();
    }

    //Sets all solenoids on robot to specfic position.
    public void setAllSolenoidsToReverse() {
        leftClimber.set(Value.kReverse);
        rightClimber.set(Value.kReverse);
        noteAimer.set(Value.kReverse);
    }
    
    public void setAllSolenoidsToForward() {
        leftClimber.set(Value.kForward);
        rightClimber.set(Value.kForward);
        noteAimer.set(Value.kForward);
    }
    
    public void turnAllSolenoidsOff() {
        leftClimber.set(Value.kOff);
        rightClimber.set(Value.kOff);
        noteAimer.set(Value.kOff);
        allSolenoidsDisabled = true;
    }
    
    public void toggleAllSolenoids() {
        leftClimber.toggle();
        rightClimber.toggle();
        noteAimer.toggle();
    }
    
    //Not reccommended to use unless necessary, compressor should always be enabled if one intends to use pneumatics.
    public void toggleCompressor() {
        if (compressor.isEnabled()) {
            compressor.disable();
            return;
        }
        compressor.enableDigital();
    }
    
    public boolean isForwardShorted() {
        if (leftClimber.isFwdSolenoidDisabled() || rightClimber.isFwdSolenoidDisabled() || noteAimer.isFwdSolenoidDisabled()) {
            return true;
        }
        return false;
    }
    
    public boolean isReverseShorted() {
        if (leftClimber.isRevSolenoidDisabled() || rightClimber.isRevSolenoidDisabled() || noteAimer.isRevSolenoidDisabled()) {
            return true;
        }
        return false;
    }
    
    private void telemetry() {
        SmartDashboard.putBoolean("All Solenoids Disabled", allSolenoidsDisabled);
    }
    
    @Override
    public void periodic() {
        //Turns pneumatics off if an electical short occurs in either the forward or reverse solenoids.
        if (isForwardShorted() || isReverseShorted()) {
            turnAllSolenoidsOff();
            compressor.disable();
        }
        
        telemetry();
    }
}
