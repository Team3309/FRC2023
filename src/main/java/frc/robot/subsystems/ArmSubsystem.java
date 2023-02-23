// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Hashtable;

import javax.swing.text.Position;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Arm;

public class ArmSubsystem extends SubsystemBase {

    public enum ArmPosition {
        Stowed,
        PickupFloor,
        PickupSubstation,
        PickupTurntable,
        ScoreHybrid,
        ScoreMid,
        ScoreTop,
    }

    public enum ArmDirection {
        Forward,
        Backwards,
    }
    
    private class ArmPose {
        public double UpperArm;
        public double LowerArm;

        public ArmPose(double upperArm, double lowerArm) {
            UpperArm = upperArm;
            LowerArm = lowerArm;
        }
    } 

    private class ArmPosePair {
        private ArmPose Forward;
        private ArmPose Backward;

        public ArmPosePair(ArmPose forward, ArmPose backward) {
            Forward = forward;
            Backward = backward;
        }

        public ArmPose getPose(ArmDirection direction) {
            if (direction == ArmDirection.Forward) {
                return Forward;
            }
            return Backward;
        }
    }

    // creating a My HashTable Dictionary
    Hashtable<ArmPosition, ArmPosePair> Poses = new Hashtable<ArmPosition, ArmPosePair>();

    private final WPI_TalonFX UpperMotor;
    private final WPI_TalonFX LowerMotor;

    private ArmPosition CurrentPosition;
    private ArmPosition DesiredPosition;

    private ArmDirection CurrentDirection;
    private ArmDirection DesiredDirection;

    public ArmSubsystem() {
        UpperMotor = new WPI_TalonFX(Constants.Arm.JOINT_A_MOTOR_ID);
        LowerMotor = new WPI_TalonFX(Constants.Arm.JOINT_B_MOTOR_ID);
        
        Constants.Arm.MOTOR_A_PID_GAINS.configureMotorPID(UpperMotor);
        Constants.Arm.MOTOR_B_PID_GAINS.configureMotorPID(LowerMotor);

        // -- Stowed Arm
        Poses.put(ArmPosition.Stowed,
            new ArmPosePair(
                new ArmPose(0, 0),
                new ArmPose(0, 0)
            )
        );

        // -- Pick up form floor
        Poses.put(ArmPosition.PickupFloor,
            new ArmPosePair(
                new ArmPose(0, 0), //Forward
                new ArmPose(0, 0)  //Backward
        ));

        // -- Pick up from Substation
        Poses.put(ArmPosition.PickupSubstation,
            new ArmPosePair(
                new ArmPose(0, 0), //Forward
                new ArmPose(0, 0)  //Backward
        ));

        // -- Pick up from Turntable
        Poses.put(ArmPosition.PickupTurntable,
            new ArmPosePair(
                new ArmPose(0, 0), //Forward
                new ArmPose(0, 0)  //Backward
        ));

        // -- Score hybrid
        Poses.put(ArmPosition.ScoreHybrid,
            new ArmPosePair(
                new ArmPose(0, 0), //Forward
                new ArmPose(0, 0)  //Backward
        ));

        // -- Score mid
        Poses.put(ArmPosition.ScoreMid,
            new ArmPosePair(
                new ArmPose(0, 0), //Forward
                new ArmPose(0, 0)  //Backward
        ));

        // -- Score top
        Poses.put(ArmPosition.ScoreTop,
            new ArmPosePair(
                new ArmPose(0, 0), //Forward
                new ArmPose(0, 0)  //Backward
        ));

    }

    public boolean IsMoving() {
        return DesiredPosition != CurrentPosition;
    }
   
    public ArmPosition getCurrentPosition() {
        return CurrentPosition;
    }

    public ArmPosition getDesiredPosition() {
        return DesiredPosition;
    }

    public void setDirection(ArmDirection direction) {
        DesiredDirection = direction;
    }

    public void setPosition(ArmPosition position) {
        DesiredPosition = position;
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (CurrentPosition == DesiredPosition && CurrentDirection == DesiredDirection) {
            return;
        }
        
        ArmPosePair posePair =  Poses.get(DesiredPosition);
        ArmPose pose = posePair.getPose(DesiredDirection);
        
        if (UpperMotor.getActiveTrajectoryPosition() == pose.UpperArm && LowerMotor.getActiveTrajectoryPosition() == pose.LowerArm) {
            CurrentPosition = DesiredPosition;
            CurrentDirection = DesiredDirection;
            return;
        }
        //TODO: make it so that the lower arm does not move untill the upper amr is passed a cerain degree 
        //TODO: vice versa for the upper arm (need solution for deadlock)

        UpperMotor.set(ControlMode.Position, pose.UpperArm);
        // LowerMotor.set(ControlMode.Position, pose.LowerArm); //keep this disabled until the upperarm is tuned 
        
    }
    
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
