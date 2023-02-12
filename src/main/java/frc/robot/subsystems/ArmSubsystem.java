// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.ArmPosition;

public class ArmSubsystem extends SubsystemBase {
    private final WPI_TalonFX jointAMotor;
    private final WPI_TalonFX jointBMotor;

    public ArmSubsystem() {
        jointAMotor = new WPI_TalonFX(Constants.Arm.JOINT_A_MOTOR_ID);
        jointBMotor = new WPI_TalonFX(Constants.Arm.JOINT_B_MOTOR_ID);    
        Constants.Arm.MOTOR_A_PID_GAINS.configureMotorPID(jointAMotor);
        Constants.Arm.MOTOR_B_PID_GAINS.configureMotorPID(jointBMotor);
    }

    public void setPosition(ArmPosition position) {
        
    }

    public ArmPosition getPosition() {
        return ArmPosition.fromCoords(0, 0);
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        
    }
    
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
