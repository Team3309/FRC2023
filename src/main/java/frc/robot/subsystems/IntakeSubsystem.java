package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.Intake.*;


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


}
