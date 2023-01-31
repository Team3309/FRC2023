package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Pneumatics;

import static frc.robot.Constants.Claw.*;

public class ClawSubsystem extends SubsystemBase {
    private final Solenoid leftClawSolenoid;
    private final Solenoid rightClawSolenoid;

    public ClawSubsystem() {
        leftClawSolenoid = new Solenoid(
            Constants.PCM_CAN_ID,
            Constants.PCM_TYPE,
            LEFT_CLAW_SOLENOID_ID
        );

        rightClawSolenoid = new Solenoid(
            Constants.PCM_CAN_ID,
            Constants.PCM_TYPE,
            RIGHT_CLAW_SOLENOID_ID
        );        
    }

    

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Air Storage Pressure", Pneumatics.getStoragePSI());
        SmartDashboard.putBoolean("Compressor State", Pneumatics.getCompressorState());
    }

}    