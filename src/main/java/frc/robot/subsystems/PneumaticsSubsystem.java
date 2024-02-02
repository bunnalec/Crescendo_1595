package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utilities.Constants.hardwareID;

public class PneumaticsSubsystem extends SubsystemBase{

    DoubleSolenoid leftClimber, rightClimber;
    Compressor compressor;

    public PneumaticsSubsystem() {
        leftClimber = new DoubleSolenoid(PneumaticsModuleType.REVPH, hardwareID.leftClimberForwardChannel, hardwareID.leftClimberReverseChannel);
        rightClimber = new DoubleSolenoid(PneumaticsModuleType.REVPH, hardwareID.rightClimberForwardChannel, hardwareID.rightClimberReverseChannel);

        compressor = new Compressor(PneumaticsModuleType.REVPH);
        compressor.enableDigital();

        setSolenoidsToReverse();
    }

    public void setSolenoidsToReverse() {
        leftClimber.set(Value.kReverse);
        rightClimber.set(Value.kReverse);
    }

    public void setSolenoidsToForward() {
        leftClimber.set(Value.kForward);
        rightClimber.set(Value.kForward);
    }

    public void turnSolenoidsOff() {
        leftClimber.set(Value.kOff);
        rightClimber.set(Value.kOff);
    }

    public void toggleSolenoids() {
        leftClimber.toggle();
        leftClimber.toggle();
    }

    //Not reccommended to use unless necessary, compressor should always be enabled if one intends to use pneumatics.
    public void toggleCompressor() {
        if (compressor.isEnabled()) {
            compressor.disable();
        }
        compressor.enableDigital();
    }

    public boolean isForwardShorted() {
        if (leftClimber.isFwdSolenoidDisabled() || rightClimber.isFwdSolenoidDisabled()) {
            return true;
        }
        return false;
    }

    public boolean isReverseShorted() {
        if (leftClimber.isRevSolenoidDisabled() || rightClimber.isRevSolenoidDisabled()) {
            return true;
        }
        return false;
    }
    
    @Override
    public void periodic() {
        //Turns pneumatics off if an electical short occurs in either the forward or reverse solenoids.
        if (isForwardShorted() || isReverseShorted()) {
            turnSolenoidsOff();
            compressor.disable();
        }
    }
}
