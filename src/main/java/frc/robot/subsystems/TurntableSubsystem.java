package frc.robot.subsystems; //brokie
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.Turntable.*;

public class TurntableSubsystem extends SubsystemBase {
    private final WPI_TalonFX turntableMotor;

    public TurntableSubsystem() {
        turntableMotor = new WPI_TalonFX(Constants.Turntable.TURNTABLE_MOTOR_ID);
        turntableMotor.configFactoryDefault();
        turntableMotor.setNeutralMode(NeutralMode.Brake);
    }
    
    public int check;

    /*
     * Start the Turntable at the motor power
     */
    public void startTurntable() {
        turntableMotor.set(ControlMode.PercentOutput, TURNTABLE_MOTOR_POWER);
    }

    /*
     * Turn off the Turntable
     */
    public void stopTurntable() {
        turntableMotor.stopMotor();
    }
    
    /*
     * Set default position
     */
    public void defaultPosition() {
        turntableMotor.set(ControlMode.Position, TURNATABLE_DEFAULT_POSITION);
    }

    /*
     * Turn to orient the cone
     */
    public void orientCone() {
        turntableMotor.set(ControlMode.Position, TURNTABLE_ORIENTATION);
    }
}
