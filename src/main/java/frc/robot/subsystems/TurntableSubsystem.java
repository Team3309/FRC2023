package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Turntable;

public class TurntableSubsystem extends SubsystemBase {
    private static final double TURNTABLE_MOTOR_POWER = 0;
    private final WPI_TalonFX turntableMotor;

    public TurntableSubsystem() {
        turntableMotor = new WPI_TalonFX(Constants.Turntable.TURNTABLE_MOTOR_ID);
        turntableMotor.configFactoryDefault();
        turntableMotor.setNeutralMode(NeutralMode.Brake);
    }
    
    /*
     * Start the Turntable at the motor power
     */
    public void starTurntable() {
        turntableMotor.set(ControlMode.PercentOutput, TURNTABLE_MOTOR_POWER);
    }

    /*
     * Turn off the conveyer
     */
    public void stopTurntable() {
        turntableMotor.stopMotor();
    }

    
    
}
