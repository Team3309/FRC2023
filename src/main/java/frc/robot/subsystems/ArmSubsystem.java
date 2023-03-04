// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Arm;

import java.util.Map;

public class ArmSubsystem extends SubsystemBase
{
    // -------------------------------------------------------------------------------------------------------------------------------------
    // -- Types
    // -------------------------------------------------------------------------------------------------------------------------------------
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
    
    private static class ArmPose {
        public double UpperArm;
        public double LowerArm;

        public ArmPose(double upperArm, double lowerArm) {
            UpperArm = upperArm;
            LowerArm = lowerArm;
        }
    }

    private static class ArmPosePair {
        private final ArmPose Forward;
        private final ArmPose Backward;

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

    
    // -------------------------------------------------------------------------------------------------------------------------------------
    // -- Static Data
    // -------------------------------------------------------------------------------------------------------------------------------------
    private static final double StowAngleFront = 10;
    private static final double StowAngleBack = 1000;
    
    private static final Map<ArmPosition, ArmPosePair> Poses = Map.ofEntries(
        
        // -- Stowed Arm
        Map.entry(ArmPosition.Stowed,
            new ArmPosePair(
                new ArmPose(0, 0),
                new ArmPose(0, 0)
        )),
        
        // -- Pick up form floor
        Map.entry(ArmPosition.PickupFloor,
            new ArmPosePair(
                new ArmPose(0, 0), //Forward
                new ArmPose(0, 0)  //Backward
        )),
        
        // -- Pick up from Substation
        Map.entry(ArmPosition.PickupSubstation,
            new ArmPosePair(
                new ArmPose(0, 0), //Forward
                new ArmPose(0, 0)  //Backward
        )),
        
        // -- Pick up from Turntable
        Map.entry(ArmPosition.PickupTurntable,
            new ArmPosePair(
                new ArmPose(0, 0), //Forward
                new ArmPose(0, 0)  //Backward
        )),
        
        // -- Score hybrid
        Map.entry(ArmPosition.ScoreHybrid,
            new ArmPosePair(
                new ArmPose(0, 0), //Forward
                new ArmPose(0, 0)  //Backward
        )),
        
        // -- Score mid
        Map.entry(ArmPosition.ScoreMid,
            new ArmPosePair(
                new ArmPose(0, 0), //Forward
                new ArmPose(0, 0)  //Backward
        )),
        
        // -- Score top
        Map.entry(ArmPosition.ScoreTop,
            new ArmPosePair(
                new ArmPose(0, 0), //Forward
                new ArmPose(0, 0)  //Backward
        ))
    );
    
    
    // -------------------------------------------------------------------------------------------------------------------------------------
    // -- Arm Subsystem
    // -------------------------------------------------------------------------------------------------------------------------------------

    private final WPI_TalonFX UpperMotor;
    private final WPI_TalonFX LowerMotor;

    private ArmPosition DesiredPosition;
    private ArmDirection DesiredDirection;
    private final Solenoid ClampSolenoid;


    public ArmSubsystem() {
        UpperMotor = new WPI_TalonFX(Constants.Arm.JOINT_A_MOTOR_ID);
        LowerMotor = new WPI_TalonFX(Constants.Arm.JOINT_B_MOTOR_ID);
        
        Constants.Arm.MOTOR_A_PID_GAINS.configureMotorPID(UpperMotor);
        Constants.Arm.MOTOR_B_PID_GAINS.configureMotorPID(LowerMotor);

        ClampSolenoid = new Solenoid(
                Constants.PCM_CAN_ID,
                Constants.PCM_TYPE,
                Constants.Arm.CLAMP_SOLENOID_ID
        );
    }
    public void setClamp (boolean open) {
        ClampSolenoid.set(open);
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
            return true;
        
        // -- Figure out the direction for the target position
        if (targetPosition < StowAngleFront)
            targetDirection = ArmDirection.Forward;
        else if (targetPosition >  StowAngleBack)
            targetDirection = ArmDirection.Backwards;
        else
            return true;

        return targetDirection != currentDirection;
    }

    private ArmPose GetPose(ArmPosition position,  ArmDirection direction)
    {
        return Poses.get(position).getPose(direction);
    }


    // -------------------------------------------------------------------------------------------------------------------------------------
    // -- Commands
    // -------------------------------------------------------------------------------------------------------------------------------------
    public Command SetDirectionCommand(ArmDirection direction)
    {
        return SetPositionAndDirectionCommand(DesiredPosition, direction);
    }
    
    public Command SetPositionCommand(ArmPosition position)
    {
        return SetPositionAndDirectionCommand(position, DesiredDirection);
    }

    public Command SetPositionAndDirectionCommand(ArmPosition position,  ArmDirection direction)
    {
        ArmPose pose = GetPose(position, direction);
        
        Command MoveUpperArm = MoveJointToTargetCommand(UpperMotor, pose.UpperArm);
        Command MoveLowerArm = MoveJointToTargetCommand(LowerMotor, pose.LowerArm);
        
        Command stowSequence = 
                 StowLowerArmCommand()
                .andThen(MoveUpperArm)
                .andThen(MoveLowerArm);
        
        Command parallelMove = Commands.parallel(MoveUpperArm, MoveLowerArm);
        
        return new ConditionalCommand(stowSequence, parallelMove, () -> DoesPoseRequireStowingLowerArm(pose))
                .beforeStarting(() -> {
                    DesiredPosition = position;
                    DesiredDirection = direction;
                });
    }

//    public Command ActuateClamp(boolean close)
//    {
//        return runOnce( () -> ClampSolenoid.set(close) );
//    }

    public Command ToggleClamp()
    {
        return runOnce(ClampSolenoid::toggle);
    }

    // -------------------------------------------------------------------------------------------------------------------------------------
    // -- Internal Commands
    // -------------------------------------------------------------------------------------------------------------------------------------
    private Command MoveJointToTargetCommand(WPI_TalonFX motor, double position)
    {
        return
             run(() -> motor.set(ControlMode.Position, position))
            .until(() -> Math.abs(motor.getActiveTrajectoryPosition() - position) < Arm.TargetThreshold);
    }
    
    private Command StowLowerArmCommand()
    {
        ArmPose pose = GetPose(ArmPosition.Stowed, ArmDirection.Forward);
        return MoveJointToTargetCommand(LowerMotor, pose.LowerArm);
    }
}
