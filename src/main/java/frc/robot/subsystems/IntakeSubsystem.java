package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Intake.*;

//How many motors are we using for the intake?
public class IntakeSubsystem extends SubsystemBase {
    private final WPI_TalonSRX topIntakeMotor;
    private final WPI_TalonSRX bottomIntakeMotor;
    
    public IntakeSubsystem() {
        topIntakeMotor = new WPI_TalonSRX(TOP_INTAKE_MOTOR_ID);
        bottomIntakeMotor = new WPI_TalonSRX(BOTTOM_INTAKE_MOTOR_ID);

        topIntakeMotor.setNeutralMode(NeutralMode.Brake); 
        bottomIntakeMotor.setNeutralMode(NeutralMode.Brake);
        
        topIntakeMotor.setInverted(true);
        bottomIntakeMotor.setInverted(false);
    }

    //activate or deactivate the rollers
    public void setTopIntakeRoller (double power) 
    {
        topIntakeMotor.set(ControlMode.PercentOutput, power);
    }

    public void setBottomIntakeRoller (double power) 
    {
        bottomIntakeMotor.set(ControlMode.PercentOutput, power);
    }

}
