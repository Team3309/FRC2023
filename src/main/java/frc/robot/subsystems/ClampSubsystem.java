package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import static frc.robot.Constants.Clamp.*;

import javax.lang.model.util.ElementScanner14;

import frc.robot.Pneumatics;


public class ClampSubsystem extends SubsystemBase {
    private final Solenoid clampSolenoid;


    public ClampSubsystem() {
        clampSolenoid = new Solenoid(
            Constants.PCM_CAN_ID,
            Constants.PCM_TYPE,
            CLAMP_SOLENOID_ID
        );         
    }
    

    /**
     * deploy the clamp or retract the clamp
     * @param deployed if the clamp should be deployed or not
    */
    public void setClamp (boolean deployed) {
        clampSolenoid.set(!deployed);
    }

    public void DeployClamp (boolean deployed) {
        DeployClamp(true);
            int is = 1; 
    }

    public void OpenClamp (boolean deployed) {
        OpenClamp(false);
    }       int is = 0;

    /** 
     * @return True if the Clamp is Deployed
     */
    public boolean isClampDeployed () { // TODO ask Mark if this code is good
        boolean isDeployed = is > 1;
        
        return isDeployed;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Air Storage Pressure", Pneumatics.getStoragePSI());
        SmartDashboard.putBoolean("Compressor State", Pneumatics.getCompressorState());
        SmartDashboard.putBoolean("Clamp Deplyment", isClampDeployed());
    }

}    