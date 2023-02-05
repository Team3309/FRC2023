package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.Intake.*;


public class IntakeSubsystem extends SubsystemBase {
    private final WPI_TalonSRX leftIntakeMotor;
    private final WPI_TalonSRX rightIntakeMotor;
    
public IntakeSubsystem() {
    leftIntakeMotor = new WPI_TalonSRX(LEFT_INTAKE_MOTOR_ID);
    rightIntakeMotor = new WPI_TalonSRX(RIGHT_INTAKE_MOTOR_ID);

    leftIntakeMotor.setNeutralMode(NeutralMode.Brake);
    rightIntakeMotor.setNeutralMode(NeutralMode.Brake);

    leftIntakeMotor.setInverted(true);
    rightIntakeMotor.setInverted(false);
    }


}
