// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import friarLib2.utility.PIDParameters;

import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

public class ArmSubsystem extends SubsystemBase
{
    // -------------------------------------------------------------------------------------------------------------------------------------
    // -- Types
    // -------------------------------------------------------------------------------------------------------------------------------------
    public enum ArmPosition
    {
        Stowed,
        PickupFloorCone,
        PickupFloorCube,
        PickupSubstationCone,
        PickupSubstationCube,
        ScoreHybrid,
        ScoreMid,
        ScoreTop,
        Test
    }

    public enum ArmDirection
    {
        Forward,
        Backward,
    }

    public enum Joint
    {
        AB,
        BC,
    }
    
    private static class ArmPose
    {
        private static final double AB_EncoderCountsPer360 = 120000;
        private static final double BC_EncoderCountsPer360 = 90000;

        public static double PositionToRadians(double position, Joint joint)
        {
            double conversion = joint == Joint.AB ? AB_EncoderCountsPer360 : BC_EncoderCountsPer360;
            return position / conversion * 2;
        }

        public static double RadiansToPosition(double radians, Joint joint)
        {
            double conversion = joint == Joint.AB ? AB_EncoderCountsPer360 : BC_EncoderCountsPer360;
            return radians * conversion / 2;
        }

        private double _UpperArm;
        private double _LowerArm;

        /**
         * @param upperArm position in radians (positive is forward)
         * @param lowerArm position in radians (positive is forward)
         */
        public ArmPose(double upperArm, double lowerArm)
        {
            _UpperArm = upperArm;
            _LowerArm = lowerArm;
        }

        public double UpperArmPosition()
        {
            return RadiansToPosition(_UpperArm, Joint.AB);
        }

        public double LowerArmPosition()
        {
            return RadiansToPosition(_LowerArm, Joint.BC);
        }
    }

    private static class ArmPosePair
    {
        private final ArmPose Forward;
        private final ArmPose Backward;

        public ArmPosePair(ArmPose forward, ArmPose backward)
        {
            Forward = forward;
            Backward = backward;
        }

        public ArmPose getPose(ArmDirection direction)
        {
            if (direction == ArmDirection.Forward)
            {
                return Forward;
            }
            return Backward;
        }
    }

    
    // -------------------------------------------------------------------------------------------------------------------------------------
    // -- Static Data
    // -------------------------------------------------------------------------------------------------------------------------------------
    private static final ArmPose StowAngleFrontPose = new ArmPose(0.171, 0);
    private static final ArmPose StowAngleBackPose = new ArmPose(-0.071, 0);

    private static final Map<ArmPosition, ArmPosePair> Poses = Map.ofEntries(

            // -- Stowed Arm
            Map.entry(ArmPosition.Stowed,
                    new ArmPosePair(
                            new ArmPose(0, 0),
                            new ArmPose(0, 0)
                    )),

            // -- Test
            Map.entry(ArmPosition.Test,
                    new ArmPosePair(
                            new ArmPose(0.2, 0.2),
                            new ArmPose(-0.2, -0.2)
                    )),

            // -- Pick up form floor
            Map.entry(ArmPosition.PickupFloorCone, // Forward is peg
                    new ArmPosePair(
                            new ArmPose(-0.056, 0.385), //Forward
                            new ArmPose(0.058, -0.466)  //Backward
                    )),
            Map.entry(ArmPosition.PickupFloorCube,
                    new ArmPosePair(
                            new ArmPose(0, 0), //Forward
                            new ArmPose(0, 0)  //Backward
                    )),

            // -- Pick up from Substation Cone
            Map.entry(ArmPosition.PickupSubstationCone,
                    new ArmPosePair(
                            new ArmPose(0.363, 0.599), //Forward
                            new ArmPose(-0.366, -0.583)  //Backward
                    )),
            Map.entry(ArmPosition.PickupSubstationCube,
                    new ArmPosePair(
                            new ArmPose(0, 0), //Forward
                            new ArmPose(0, 0)  //Backward
                    )),

            // -- Score hybrid
            Map.entry(ArmPosition.ScoreHybrid,
                    new ArmPosePair(
                            new ArmPose(0.147, 0.761), //Forward
                            new ArmPose(0, -0.53)  //Backward
                    )),

            // -- Score mid
            Map.entry(ArmPosition.ScoreMid,
                    new ArmPosePair(
                            new ArmPose(0.351, 0.706), //Forward
                            new ArmPose(-0.256, -0.398)  //Backward
                    )),

            // -- Score top
            Map.entry(ArmPosition.ScoreTop,
                    new ArmPosePair(
                            new ArmPose(0, 0), //Forward
                            new ArmPose(-0.461, -0.768)  //Backward
                    ))
    );
    
    // -------------------------------------------------------------------------------------------------------------------------------------
    // -- Arm Subsystem
    // -------------------------------------------------------------------------------------------------------------------------------------

    private final boolean AlwaysStow = false;

    private final WPI_TalonFX Motor_AB;
    private final WPI_TalonFX Motor_BC;
    private final DoubleSolenoid ClampSolenoid;

    private ArmPosition DesiredPosition = ArmPosition.Stowed;
    private ArmDirection DesiredDirection = ArmDirection.Backward;
    
    public ArmSubsystem()
    {
        // TODO: Update soft limits to use radians!!
        Motor_AB = ConfigureMotor(Constants.Arm.AB_MOTOR_ID,
                Constants.Arm.JOINT_AB_MOTOR_PID,
                0.75,
                ArmPose.RadiansToPosition(0.523, Joint.AB),
                ArmPose.RadiansToPosition(-0.5, Joint.AB),
                6000,
                20000);

        Motor_BC = ConfigureMotor(Constants.Arm.BC_MOTOR_ID,
                Constants.Arm.JOINT_BC_MOTOR_PID,
                1.0,
                ArmPose.RadiansToPosition(1, Joint.BC),
                ArmPose.RadiansToPosition(-1, Joint.BC),
                5000,
                20000);
        
        ClampSolenoid = new DoubleSolenoid(
                Constants.PCM_TYPE,
                1,
                14

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
        motor.config_kP(pidConstants.GetSlotIndex(), pidConstants.GetP(), Constants.TIMEOUT_MS);
        motor.config_kI(pidConstants.GetSlotIndex(), pidConstants.GetI(), Constants.TIMEOUT_MS);
        motor.config_kD(pidConstants.GetSlotIndex(), pidConstants.GetD(), Constants.TIMEOUT_MS);
        motor.config_kF(pidConstants.GetSlotIndex(), pidConstants.GetF(), Constants.TIMEOUT_MS);

        // -- Ramp speeds
        motor.configMotionCruiseVelocity(cruise, Constants.TIMEOUT_MS);
        motor.configMotionAcceleration(acceleration, Constants.TIMEOUT_MS); // unloaded = 10,000
        motor.configMotionSCurveStrength(8);

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
        double targetPosition = targetPose.UpperArmPosition();

        System.out.printf("Current: %.2f Target: %.2f\n", currentPosition, targetPosition);

        ArmDirection currentDirection;
        ArmDirection targetDirection;

        // -- Figure out the direction for the current position
        if (currentPosition > StowAngleFrontPose.UpperArmPosition())
            currentDirection = ArmDirection.Forward;
        else if (currentPosition < StowAngleBackPose.UpperArmPosition())
            currentDirection = ArmDirection.Backward;
        else
            return true;
        
        // -- Figure out the direction for the target position
        if (targetPosition > StowAngleFrontPose.UpperArmPosition())
            targetDirection = ArmDirection.Forward;
        else if (targetPosition < StowAngleBackPose.UpperArmPosition())
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

    public Command Command_SetPositionAndDirection(ArmPosition position, ArmDirection direction)
    {
        return Commands.sequence(new PrintCommand(String.format("SetPositionAndDirection %s %s\n", position.name(), direction.name()))
                , runOnce(() ->
                {
                    DesiredPosition = position;
                    DesiredDirection = direction;
                })
                , Command_MoveArmToDesired()
        );
    }

    public Command Command_SetDirection(ArmDirection direction)
    {
        return Commands.sequence(new PrintCommand(String.format("SetDirection %s\n", direction.name()))
                , runOnce(() -> DesiredDirection = direction)
                , Command_MoveArmToDesired()
        );
    }

    public Command Command_SetPosition(ArmPosition position)
    {
        return Commands.sequence(new PrintCommand(String.format("SetPosition %s\n", position.name()))
                , runOnce(() -> DesiredPosition = position)
                , Command_MoveArmToDesired()
        );
    }
    
    public Command Command_ActuateClamp(DoubleSolenoid.Value value)
    {
        return runOnce( () -> ClampSolenoid.set(value) );
    }

    public Command Command_ToggleClamp()
    {
        return runOnce(ClampSolenoid::toggle);
    }

    public Command Command_ZeroArm()
    {
        return runOnce(() ->
        {
            System.out.println("Zeroing Arm");
            Motor_AB.setSelectedSensorPosition(0, 0, Constants.TIMEOUT_MS);
            Motor_BC.setSelectedSensorPosition(0, 0, Constants.TIMEOUT_MS);
        }).ignoringDisable(true);
    }

    public Command Command_OutputArmPosition()
    {
        return runOnce(() ->
                System.out.printf("Upper: %.3f (%.0f), Lower: %.3f (%.0f)\n"
                        , ArmPose.PositionToRadians(Motor_AB.getSelectedSensorPosition(), Joint.AB)
                        , Motor_AB.getSelectedSensorPosition()
                        , ArmPose.PositionToRadians(Motor_BC.getSelectedSensorPosition(), Joint.BC)
                        , Motor_BC.getSelectedSensorPosition())
        ).ignoringDisable(true);
    }

    // -------------------------------------------------------------------------------------------------------------------------------------
    // -- Internal Commands
    // -------------------------------------------------------------------------------------------------------------------------------------
    
    // -- Calculates a safe trajectory to move the arm to the desired position/direction.
    public Command Command_MoveArmToDesired()
    {
        Command stowSequence = Commands.sequence(new PrintCommand("Stow Arm Sequence")
            , Command_StowClaw()
            , Command_ActuateArmToDesired_UNSAFE(true, false) // Move arm first, keeping the claw stowed
            , Command_ActuateArmToDesired_UNSAFE(false, true) // Move claw now that arm is in place
        );

        Command parallelMove = Commands.sequence(new PrintCommand("Parallel Arm Sequence")
            , Command_ActuateArmToDesired_UNSAFE(true, true)
        );

        return new ConditionalCommand(
            stowSequence,
            parallelMove,
            () -> AlwaysStow || DoesPoseRequireStowingLowerArm(GetPose(DesiredPosition, DesiredDirection))
        );
    }
    
    // -- Does the actual work of actuating the motors.
    //    This command is unsafe, it does not handle preventing collisions. Use SAFE for that.
    private Command Command_ActuateArmToDesired_UNSAFE(boolean moveAB, boolean moveBC)
    {
        AtomicReference<ArmPose> DesiredPose = new AtomicReference<>();
        
        Command Debug = runOnce(() -> System.out.printf("Move Joint\n  AB %.2f -> %.2f\n  BC: %.2f -> %.2f\n"
                , Motor_AB.getSelectedSensorPosition(0)
                , DesiredPose.get().UpperArmPosition()
                , Motor_BC.getSelectedSensorPosition(0)
                , DesiredPose.get().LowerArmPosition()
            ));

        
        Command Move =
            run(() ->
            {
                if (moveAB)
                {
                    Motor_AB.set(ControlMode.MotionMagic, DesiredPose.get().UpperArmPosition());
                }
    
                if (moveBC)
                {
                    Motor_BC.set(ControlMode.MotionMagic, DesiredPose.get().LowerArmPosition());
                }
            })
            .until(() ->
            {
                boolean abAtTarget = true;
                boolean bcAtTarget = true;

                if (moveAB)
                {
                    //System.out.printf("Motor %d at position %f\n", Motor_AB.getDeviceID(), Motor_AB.getActiveTrajectoryPosition());
                    abAtTarget = Math.abs(Motor_AB.getActiveTrajectoryPosition() - DesiredPose.get().UpperArmPosition()) < Arm.TargetThreshold;
                }

                if (moveBC)
                {
                    //System.out.printf("Motor %d at position %f\n", Motor_BC.getDeviceID(), Motor_BC.getActiveTrajectoryPosition());
                    bcAtTarget = Math.abs(Motor_BC.getActiveTrajectoryPosition() - DesiredPose.get().LowerArmPosition()) < Arm.TargetThreshold;
                }

                return abAtTarget && bcAtTarget;
            });
        
        return runOnce(() -> DesiredPose.set(GetPose(DesiredPosition, DesiredDirection)))
                .andThen(Debug)
                .andThen(Move);
            
    }
    
    // -- Store off the current desired position, move the claw to the stowed position, then restore the desired position
    private Command Command_StowClaw()
    {
        AtomicReference<ArmPosition> PreviousDesiredPosition = new AtomicReference<>();
        
        return Commands.sequence(
                runOnce(() ->
                {
                    PreviousDesiredPosition.set(DesiredPosition); // Cache off the current desired position
                    DesiredPosition = ArmPosition.Stowed; // Update the desired position to stow the claw 
                })
                , Command_ActuateArmToDesired_UNSAFE(false, true) // Move the claw
                , runOnce(() -> DesiredPosition = PreviousDesiredPosition.get()) // Restore cached desired position
        );
    }
}
