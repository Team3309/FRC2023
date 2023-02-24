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
        Inside,
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

    private static double StowAngleFront = 10;
    private static double StowAngleBack = 1000;

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


    /**
     * Calculates if moving to a pose will cause the upper arm to pass through the robot, thus requiring the lower arm to be stowed.
     */
    private boolean DoesPoseRequireStowingLowerArm(ArmPose targetPose)
    {
        double currentPosition = UpperMotor.getActiveTrajectoryPosition();
        double targetPosition = targetPose.UpperArm;

        ArmDirection currentDirection;
        ArmDirection targetDirection;

        // -- Figure out the direction for the current position
        if (currentPosition < StowAngleFront)
            currentDirection = ArmDirection.Forward;
        else if (currentPosition > StowAngleBack)
            currentDirection = ArmDirection.Backwards;
        else
            currentDirection = ArmDirection.Inside;


        // -- Figure out the direction for the target position
        if (targetPosition < StowAngleFront)
            targetDirection = ArmDirection.Forward;
        else if (targetPosition >  StowAngleBack)
            targetDirection = ArmDirection.Backwards;
        else
            targetDirection = ArmDirection.Inside;

        if (targetDirection == ArmDirection.Inside)
        {
            return true;
        }

        return targetDirection != currentDirection;
    }

    private ArmPose GetCurrentPose()
    {
        return Poses.get(CurrentPosition).getPose(CurrentDirection);
    }

    private ArmPose GetDesiredPose()
    {
        return Poses.get(DesiredPosition).getPose(DesiredDirection);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (CurrentPosition == DesiredPosition && CurrentDirection == DesiredDirection) {
            return;
        }

        ArmPose pose = GetDesiredPose();
        
        if (UpperMotor.getActiveTrajectoryPosition() == pose.UpperArm && LowerMotor.getActiveTrajectoryPosition() == pose.LowerArm) {
            CurrentPosition = DesiredPosition;
            CurrentDirection = DesiredDirection;
            return;
        }

        boolean stow = DoesPoseRequireStowingLowerArm(pose);
        if (stow)
        {
            ArmPose stowPose = Poses.get(ArmPosition.Stowed).getPose(ArmDirection.Forward);
            LowerMotor.set(ControlMode.Position, stowPose.LowerArm);
            if (LowerMotor.getActiveTrajectoryPosition() != stowPose.LowerArm)
            {
                return;
            }
        }

        UpperMotor.set(ControlMode.Position, pose.UpperArm);

        // TODO: This code is fragile and we should think of a better way to do this
        if (!stow) {
            // LowerMotor.set(ControlMode.Position, pose.LowerArm); //keep this disabled until the upperarm is tuned
        }
        else {
            if (!DoesPoseRequireStowingLowerArm(pose))
            {
                // LowerMotor.set(ControlMode.Position, pose.LowerArm); //keep this disabled until the upperarm is tuned
            }
        }
    }
    
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
