// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import friarLib2.utility.PIDParameters;
import pabeles.concurrency.ConcurrencyOps;

import java.util.HashMap;
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
        Backward,
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
    private static final double StowAngleFront = 11000;
    private static final double StowAngleBack = -6000;
    
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
                            new ArmPose(9500, 0), //Forward
                            new ArmPose(4000, 0)  //Backward
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
                            new ArmPose(10000, 0), //Forward
                            new ArmPose(-10000, 0)  //Backward
                    )),

            // -- Score mid
            Map.entry(ArmPosition.ScoreMid,
                    new ArmPosePair(
                            new ArmPose(20000, 0), //Forward
                            new ArmPose(-15000, 0)  //Backward
                    )),

            // -- Score top
            Map.entry(ArmPosition.ScoreTop,
                    new ArmPosePair(
                            new ArmPose(-30000, 0), //Forward
                            new ArmPose(-30000, 0)  //Backward
                    ))
    );
    
    // -------------------------------------------------------------------------------------------------------------------------------------
    // -- Arm Subsystem
    // -------------------------------------------------------------------------------------------------------------------------------------

    private final boolean AlwaysStow = true;

    private final WPI_TalonFX Motor_AB;
    private final WPI_TalonFX Motor_BC;

    private ArmPosition DesiredPosition = ArmPosition.Stowed;
    private ArmDirection DesiredDirection = ArmDirection.Forward;
    private final Solenoid ClampSolenoid;

    public ArmSubsystem()
    {
        Motor_AB = ConfigureMotor(Constants.Arm.AB_MOTOR_ID,
                Constants.Arm.JOINT_AB_MOTOR_PID,
                0.75,
                35000,
                -33500,
                6000,
                20000);
        
        Motor_BC = ConfigureMotor(Constants.Arm.BC_MOTOR_ID,
                Constants.Arm.JOINT_BC_MOTOR_PID,
                1.0,
                30000,
                -30000,
                5000,
                20000);
        
        ClampSolenoid = new Solenoid(
                Constants.PCM_CAN_ID,
                Constants.PCM_TYPE,
                Constants.Arm.CLAMP_SOLENOID_ID
        );
    }

    private WPI_TalonFX ConfigureMotor(
              int motorId
            , PIDParameters pidConstants
            , double peakOutput
            , double softLimitForward
            , double softLimitReverse
            , double acceleration
            , double cruise)
    {
        WPI_TalonFX motor = new WPI_TalonFX(motorId);
        
        // --Factory default hardware to prevent unexpected behavior
        motor.configFactoryDefault();

        // -- Configure Sensor Source for Pirmary PID
        motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.TIMEOUT_MS);

		// -- set deadband to super small 0.001 (0.1 %).
        //    The default deadband is 0.04 (4 %)
        motor.configNeutralDeadband(0.001, Constants.TIMEOUT_MS);
        
        motor.setSensorPhase(false);
        motor.setInverted(true);

        // -- Set relevant frame periods to be at least as fast as periodic rate
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.TIMEOUT_MS);
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.TIMEOUT_MS);

        // -- Set the peak and nominal outputs
        motor.configNominalOutputForward(0, Constants.TIMEOUT_MS);
        motor.configNominalOutputReverse(0, Constants.TIMEOUT_MS);
        motor.configPeakOutputForward(peakOutput, Constants.TIMEOUT_MS);
        motor.configPeakOutputReverse(-peakOutput, Constants.TIMEOUT_MS);

        // -- Soft limits
        motor.configForwardSoftLimitThreshold(softLimitForward);
        motor.configReverseSoftLimitThreshold(softLimitReverse);
        motor.configForwardSoftLimitEnable(true);
        motor.configReverseSoftLimitEnable(true);
        
        // -- Motion Magic
        motor.selectProfileSlot(pidConstants.GetSlotIndex(), 0);
        motor.config_kF(pidConstants.GetSlotIndex(), pidConstants.GetD(), Constants.TIMEOUT_MS);
        motor.config_kP(pidConstants.GetSlotIndex(), pidConstants.GetD(), Constants.TIMEOUT_MS);
        motor.config_kI(pidConstants.GetSlotIndex(), pidConstants.GetD(), Constants.TIMEOUT_MS);
        motor.config_kD(pidConstants.GetSlotIndex(), pidConstants.GetD(), Constants.TIMEOUT_MS);
        
        // -- Ramp speeds
        motor.configMotionCruiseVelocity(cruise, Constants.TIMEOUT_MS);
        motor.configMotionAcceleration(acceleration, Constants.TIMEOUT_MS); // unloaded = 10,000

        // -- Zero the sensor once on robot boot up
        motor.setSelectedSensorPosition(0, 0, Constants.TIMEOUT_MS);
        
        return motor;
    }
    
    /**
     * Calculates if moving to a pose will cause the upper arm to pass through the robot, thus requiring the lower arm to be stowed.
     */
    private boolean DoesPoseRequireStowingLowerArm(ArmPose targetPose)
    {
        double currentPosition = Motor_AB.getActiveTrajectoryPosition();
        double targetPosition = targetPose.UpperArm;

        ArmDirection currentDirection;
        ArmDirection targetDirection;

        // -- Figure out the direction for the current position
        if (currentPosition < StowAngleFront)
            currentDirection = ArmDirection.Forward;
        else if (currentPosition > StowAngleBack)
            currentDirection = ArmDirection.Backward;
        else
            return true;
        
        // -- Figure out the direction for the target position
        if (targetPosition < StowAngleFront)
            targetDirection = ArmDirection.Forward;
        else if (targetPosition >  StowAngleBack)
            targetDirection = ArmDirection.Backward;
        else
            return true;

        return targetDirection != currentDirection;
    }

    private ArmPose GetPose(ArmPosition position, ArmDirection direction)
    {
        if (!Poses.containsKey(position))
        {
            System.out.println("Pose not found for position " + position.name());
            return new ArmPose(0, 0);
        }

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
        System.out.printf("Pose - Upper: %f Lower: %f", pose.UpperArm, pose.LowerArm);

        Command MoveUpperArm = MoveJointToTargetCommand(Motor_AB, pose.UpperArm);
        Command MoveLowerArm = MoveJointToTargetCommand(Motor_BC, pose.LowerArm);

        // Testing upper arm only
        if (true)
        {
            return MoveUpperArm;
        }

        Command stowSequence = Commands.sequence(
                 StowLowerArmCommand()
                , MoveUpperArm
                //, MoveLowerArm
        );

        if (AlwaysStow)
        {
            return stowSequence;
        }
        
        Command parallelMove = Commands.parallel(MoveUpperArm, MoveLowerArm);
        
        return new ConditionalCommand(stowSequence, parallelMove, () -> DoesPoseRequireStowingLowerArm(pose))
                .beforeStarting(() -> {
                    DesiredPosition = position;
                    DesiredDirection = direction;
                });
    }

    public Command ActuateClamp(boolean close) { return runOnce( () -> ClampSolenoid.set(!close) ); }

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
             run(() ->
             {
                 System.out.printf("Driving motor %d to position %f", motor.getDeviceID(), position);
                 //motor.set(ControlMode.MotionMagic, position);
             })
            .until(() ->
            {
                System.out.printf("Motor %d at position %f", motor.getDeviceID(), motor.getActiveTrajectoryPosition());
                return Math.abs(motor.getActiveTrajectoryPosition() - position) < Arm.TargetThreshold;
            });
    }
    
    private Command StowLowerArmCommand()
    {
        ArmPose pose = GetPose(ArmPosition.Stowed, ArmDirection.Forward);
        return MoveJointToTargetCommand(Motor_BC, pose.LowerArm);
    }
}
