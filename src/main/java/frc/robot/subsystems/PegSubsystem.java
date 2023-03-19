package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PegSubsystem extends SubsystemBase
{
    private final DoubleSolenoid Solenoid;
    
    public PegSubsystem()
    {
        Solenoid = new DoubleSolenoid(
            Constants.PCM_TYPE,
            0,
            15
        );
        
        Solenoid.set(DoubleSolenoid.Value.kReverse);
    }
    
    public CommandBase Command_ExtendWhile()
    {
        return startEnd(
                () -> Solenoid.set(DoubleSolenoid.Value.kForward),
                () -> Solenoid.set(DoubleSolenoid.Value.kReverse)
        );
    }
}
