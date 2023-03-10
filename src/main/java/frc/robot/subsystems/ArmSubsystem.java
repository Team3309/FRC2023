// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import friarLib2.utility.PIDParameters;

import java.util.Map;

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
        PickupTurntable,
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
        public double UpperArm;
        public double LowerArm;

        public ArmPose(double upperArm, double lowerArm)
        {
            UpperArm = upperArm;
            LowerArm = lowerArm;
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
    private static final double StowAngleFront = 9900;
    private static final double StowAngleBack = -6000;
    
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
                            new ArmPose(10000, 5000),
                            new ArmPose(-10000, -5000)
                    )),

            // -- Pick up form floor
            Map.entry(ArmPosition.PickupFloorCone,
                    new ArmPosePair(
                            new ArmPose(4100, 28000), //Forward
                            new ArmPose(3000, -21000)  //Backward
                    )),

            // -- Pick up form floor
            Map.entry(ArmPosition.PickupFloorCube,
                    new ArmPosePair(
                            new ArmPose(11000, 43000), //Forward
                            new ArmPose(-2500, -31000)  //Backward
                    )),

            // -- Pick up from Substation Cone
            Map.entry(ArmPosition.PickupSubstationCone,
                    new ArmPosePair(
                            new ArmPose(23000, 31000), //Forward
                            new ArmPose(-23000, 27000)  //Backward
                    )),

            // -- Pick up from Substation Cube
            Map.entry(ArmPosition.PickupSubstationCube,
                    new ArmPosePair(
                            new ArmPose(23000, 30000), //Forward
                            new ArmPose(-24000, -33000)  //Backward
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
                            new ArmPose(9600, 30000), //Forward
                            new ArmPose(-300, -21200)  //Backward
                    )),

            // -- Score mid
            Map.entry(ArmPosition.ScoreMid,
                    new ArmPosePair(
                            new ArmPose(25000, 35000), //Forward
                            new ArmPose(-20000, -20500)  //Backward
                    )),

            // -- Score top
            Map.entry(ArmPosition.ScoreTop,
                    new ArmPosePair(
                            new ArmPose(-32300, -40250), //Forward
                            new ArmPose(-32300, -40250)  //Backward
                    ))
    );
    
    // -------------------------------------------------------------------------------------------------------------------------------------
    // -- Arm Subsystem
    // -------------------------------------------------------------------------------------------------------------------------------------

    private final boolean AlwaysStow = true;

    private final WPI_TalonFX Motor_AB;
    private final WPI_TalonFX Motor_BC;
    private final Solenoid ClampSolenoid;

    private ArmPosition DesiredPosition = ArmPosition.Stowed;
    private ArmDirection DesiredDirection = ArmDirection.Forward;
    private ArmPose DesiredPose = null;

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

        ClampSolenoid = null;
//        ClampSolenoid = new Solenoid(
//                Constants.PCM_CAN_ID,
//                Constants.PCM_TYPE,
//                Constants.Arm.CLAMP_SOLENOID_ID
//        );
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
        double targetPosition = targetPose.UpperArm;

        System.out.printf("Current: %.2f Target: %.2f\n", currentPosition, targetPosition);

        ArmDirection currentDirection;
        ArmDirection targetDirection;

        // -- Figure out the direction for the current position
        if (currentPosition > StowAngleFront)
            currentDirection = ArmDirection.Forward;
        else if (currentPosition < StowAngleBack)
            currentDirection = ArmDirection.Backward;
        else
            return true;
        
        // -- Figure out the direction for the target position
        if (targetPosition > StowAngleFront)
            targetDirection = ArmDirection.Forward;
        else if (targetPosition <  StowAngleBack)
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

    private WPI_TalonFX GetMotorForJoint(Joint joint)
    {
        return joint == Joint.AB ? Motor_AB : Motor_BC;
    }

    private double GetPositionForPoseJoint(ArmPose pose, Joint joint)
    {
        return joint == Joint.AB ? pose.UpperArm : pose.LowerArm;
    }

    // -------------------------------------------------------------------------------------------------------------------------------------
    // -- Commands
    // -------------------------------------------------------------------------------------------------------------------------------------

    public Command SetPositionAndDirectionCommand(ArmPosition position, ArmDirection direction)
    {
        return Commands.sequence(new PrintCommand(String.format("SetArmAndDirectionCommand %s %s\n", position.name(), direction.name()))
                , runOnce(() ->
                {
                    DesiredPosition = position;
                    DesiredDirection = direction;
                })
                , GoToDesiredPositionAndDirectionCommand()
        );
    }

    public Command SetDirectionCommand(ArmDirection direction)
    {
        return Commands.sequence(new PrintCommand(String.format("SetDirectionCommand %s\n", direction.name()))
                , runOnce(() -> DesiredDirection = direction)
                , GoToDesiredPositionAndDirectionCommand()
        );
    }

    public Command SetPositionCommand(ArmPosition position)
    {
        return Commands.sequence(new PrintCommand(String.format("SetPositionCommand %s\n", position.name()))
                , runOnce(() -> DesiredPosition = position)
                , GoToDesiredPositionAndDirectionCommand()
        );
    }

    public Command ActuateClampCommand(boolean close)
    {
        return runOnce( () -> ClampSolenoid.set(!close) );
    }

    public Command ToggleClampCommand()
    {
        return runOnce(ClampSolenoid::toggle);
    }

    public Command ZeroArmCommand()
    {
        return runOnce(() ->
        {
            System.out.println("Zeroing Arm");
            Motor_AB.setSelectedSensorPosition(0, 0, Constants.TIMEOUT_MS);
            Motor_BC.setSelectedSensorPosition(0, 0, Constants.TIMEOUT_MS);
        }).ignoringDisable(true);
    }

    public Command OutputArmPositionCommand()
    {
        return runOnce(() ->
                System.out.printf("Upper: %.2f, Lower: %.2f\n"
                        , Motor_AB.getSelectedSensorPosition()
                        , Motor_BC.getSelectedSensorPosition()))
                .ignoringDisable(true);
    }

    // -------------------------------------------------------------------------------------------------------------------------------------
    // -- Internal Commands
    // -------------------------------------------------------------------------------------------------------------------------------------
    public Command GoToDesiredPositionAndDirectionCommand()
    {
        Command stowSequence = Commands.sequence(new PrintCommand("Stow Arm Sequence")
                , CommandStowArmCommand()
                , MoveJointToDesiredPose(Joint.AB)
                , MoveJointToDesiredPose(Joint.BC)
        );

        Command parallelMove = Commands.parallel(new PrintCommand("Parallel Arm Sequence")
                , MoveArmToDesiredPose()
        );

        return Commands.sequence(
                runOnce(() -> DesiredPose = GetPose(DesiredPosition, DesiredDirection))
                , new ConditionalCommand(
                        stowSequence,
                        parallelMove,
                        () -> AlwaysStow || DoesPoseRequireStowingLowerArm(DesiredPose)
                ));
    }

    private Command MoveJointToDesiredPose(Joint joint)
    {
        return
             run(() ->
             {
                 var motor = GetMotorForJoint(joint);
                 var position = GetPositionForPoseJoint(DesiredPose, joint);

                 //System.out.printf("Driving motor %d to position %f\n", motor.getDeviceID(), position);
                 motor.set(ControlMode.MotionMagic, position);
             })
            .until(() ->
            {
                var motor = GetMotorForJoint(joint);
                var position = GetPositionForPoseJoint(DesiredPose, joint);

                //System.out.printf("Motor %d at position %f\n", motor.getDeviceID(), motor.getActiveTrajectoryPosition());
                return Math.abs(motor.getActiveTrajectoryPosition() - position) < Arm.TargetThreshold;
            });
    }

    // -- Moves the arm and the claw at the same time together
    private Command MoveArmToDesiredPose()
    {
        return
                run(() -> {
                    Motor_AB.set(ControlMode.MotionMagic, DesiredPose.UpperArm);
                    Motor_BC.set(ControlMode.MotionMagic, DesiredPose.LowerArm);
                })
                .until(() ->   Math.abs(Motor_AB.getActiveTrajectoryPosition() - DesiredPose.UpperArm) < Arm.TargetThreshold
                            && Math.abs(Motor_BC.getActiveTrajectoryPosition() - DesiredPose.LowerArm) < Arm.TargetThreshold);
    }

    private Command CommandStowArmCommand()
    {
        return Commands.sequence(
                  runOnce(() -> DesiredPosition = ArmPosition.Stowed)
                , MoveArmToDesiredPose()
                );
    }
}
