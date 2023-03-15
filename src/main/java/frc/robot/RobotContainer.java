// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.auto.autos.AutoBalancePath;
import frc.robot.commands.drive.DriveTeleop;
import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.commands.drive.TurnInDirectionOfTarget;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

import java.util.HashMap;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    // -- Subsystems
    private final ArmSubsystem Arm = new ArmSubsystem();
    private final DriveSubsystem Drive = new DriveSubsystem();

    // -- Auto
    private SwerveAutoBuilder AutoBuilder = null;
    private final SendableChooser<Command> AutoChooser = new SendableChooser<>();


    public RobotContainer()
    {
        ConfigureBindings();
        SetDefaultCommands();
        ConfigureSwerveAutoBuilder();
        ConfigureAutoCommands();
    }


    public Command GetAutonomousCommand()
    {
        return AutoChooser.getSelected();
    }

    public void StowArm()
    {
        Arm.Command_SetPosition(ArmSubsystem.ArmPosition.Stowed).schedule();
    }

    private void ConfigureBindings()
    {
        // ----------------------------------------------------------------------------------------
        // -- Driver
        // ----------------------------------------------------------------------------------------

        // -- Clamp
//        new Trigger(OI.rightStick::getTrigger).onTrue(Arm.ActuateClampCommand(DoubleSolenoid.Value.kForward));
//        new Trigger(OI.rightStick::getTop).onTrue(Arm.ActuateClampCommand(DoubleSolenoid.Value.kReverse));
        new Trigger(OI.operatorController::IsLeftTriggerPressed).onTrue(Arm.Command_ActuateClamp(DoubleSolenoid.Value.kForward));
        new Trigger(OI.operatorController::IsRightTriggerPressed).onTrue(Arm.Command_ActuateClamp(DoubleSolenoid.Value.kReverse));
//        new Trigger(OI.rightStick::getTrigger).onTrue(Arm.ToggleClampCommand());

        // -- Auto Turn
        new Trigger(OI.rightStick::getTrigger).whileTrue(new TurnInDirectionOfTarget(Drive));

        //Zero IMU
        new Trigger(OI.leftStick::getTop).whileTrue(new InstantCommand(IMU::zeroIMU));

        //Zeroing
        new Trigger(OI.operatorController::getLeftStickButton).whileTrue(Arm.Command_ZeroArm().ignoringDisable(true));

        //new Trigger(OI.leftStick::getTrigger).onTrue(Drive.Command_AutoBalance());





        // ----------------------------------------------------------------------------------------
        // -- Operator
        // ----------------------------------------------------------------------------------------

        // -- Arm
        new Trigger(OI.operatorController::getAButton).onTrue(Arm.Command_SetPosition(ArmSubsystem.ArmPosition.Stowed));
        new Trigger(OI.operatorController::getBButton).onTrue(Arm.Command_SetPosition(ArmSubsystem.ArmPosition.ScoreHybrid));
        //new Trigger(OI.operatorController::getBButton).onTrue(Arm.Command_SetPosition(ArmSubsystem.ArmPosition.Test));
        new Trigger(OI.operatorController::getXButton).onTrue(Arm.Command_SetPosition(ArmSubsystem.ArmPosition.ScoreMid));
        new Trigger(OI.operatorController::getYButton).onTrue(Arm.Command_SetPosition(ArmSubsystem.ArmPosition.ScoreTop));
        new Trigger(() -> OI.operatorController.getPOV() == 0).onTrue(Arm.Command_SetPosition(ArmSubsystem.ArmPosition.PickupSubstationCone)); //D-pad up
        new Trigger(() -> OI.operatorController.getPOV() == 90).onTrue(Arm.Command_SetPosition(ArmSubsystem.ArmPosition.PickupFloorCone)); //D-pad right
        new Trigger(() -> OI.operatorController.getPOV() == 180).onTrue(Arm.Command_SetPosition(ArmSubsystem.ArmPosition.PickupFloorCube)); //D-pad down
        new Trigger(() -> OI.operatorController.getPOV() == 270).onTrue(Arm.Command_SetPosition(ArmSubsystem.ArmPosition.PickupSubstationCube)); //D-pad left


        //new Trigger(OI.operatorController::getRightBumper).onTrue(Arm.Command_SetDirection(ArmSubsystem.ArmDirection.Forward));
        new Trigger(OI.operatorController::getLeftBumper).onTrue(Arm.Command_SetDirection(ArmSubsystem.ArmDirection.Backward));

        //new Trigger(OI.operatorController::getBackButton).onTrue(Arm.Command_OutputArmPosition());

        // -- Intake
        //new Trigger(OI.XboxController::leftBumper).whileTrue(new ActivateRollers());

        // -- AutoBalance
        //new Trigger(OI.rightStick::getTrigger).onTrue(Drive.Command_AutoBalance());

        // -- Vision
        new Trigger(OI.operatorController::getBackButton).onTrue(LimelightVision.SetPipelineCommand(0));
        new Trigger(OI.operatorController::getStartButton).onTrue(LimelightVision.SetPipelineCommand(1));

        // -- Turntable

        // -- Reset Odometry

        // -- Re-zero the arm (for debugging)
        //new Trigger(OI.operatorController::getStartButton).whileTrue(Arm.Command_ZeroArm());
    }


    private void SetDefaultCommands()
    {
        Drive.setDefaultCommand(new DriveTeleop(Drive));
        
//        Arm.setDefaultCommand(
//             Arm.Command_SetPosition(ArmSubsystem.ArmPosition.Stowed)
//            .alongWith(Commands.run(() -> {}))
//        );
    }


    private void ConfigureSwerveAutoBuilder()
    {
        // -- Map Path Planner events to Commands
        HashMap<String, Command> eventMap = new HashMap<>();

        eventMap.put("BalanceForward", Drive.Command_AutoBalance());

        eventMap.put("Arm_ScoreTop_Forward", Arm.Command_SetPositionAndDirection(ArmSubsystem.ArmPosition.ScoreTop, ArmSubsystem.ArmDirection.Forward));
        eventMap.put("Arm_ScoreTop_Backward", Arm.Command_SetPositionAndDirection(ArmSubsystem.ArmPosition.ScoreTop, ArmSubsystem.ArmDirection.Backward));
        eventMap.put("Arm_ScoreMid_Forward", Arm.Command_SetPositionAndDirection(ArmSubsystem.ArmPosition.ScoreMid, ArmSubsystem.ArmDirection.Forward));
        eventMap.put("Arm_ScoreMid_Backward", Arm.Command_SetPositionAndDirection(ArmSubsystem.ArmPosition.ScoreMid, ArmSubsystem.ArmDirection.Backward));
        eventMap.put("Arm_ScoreHybrid_Forward", Arm.Command_SetPositionAndDirection(ArmSubsystem.ArmPosition.ScoreHybrid, ArmSubsystem.ArmDirection.Forward));
        eventMap.put("Arm_PickupHybrid_Backward", Arm.Command_SetPositionAndDirection(ArmSubsystem.ArmPosition.ScoreHybrid, ArmSubsystem.ArmDirection.Backward));
        eventMap.put("Arm_PickupFloor_Backward", Arm.Command_SetPositionAndDirection(ArmSubsystem.ArmPosition.PickupFloorCone, ArmSubsystem.ArmDirection.Backward));
        eventMap.put("Arm_Stow", Arm.Command_SetPositionAndDirection(ArmSubsystem.ArmPosition.Stowed, ArmSubsystem.ArmDirection.Forward));
        eventMap.put("Wait", new WaitCommand(0.5));


        eventMap.put("Clamp_Close", Arm.Command_ActuateClamp(DoubleSolenoid.Value.kForward));
        eventMap.put("Clamp_Open", Arm.Command_ActuateClamp(DoubleSolenoid.Value.kReverse));

        // -- Builder
        AutoBuilder = new SwerveAutoBuilder(
                Drive::GetPose, // Pose2d supplier
                Drive::HardResetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
                Drive.GetKinematics(), // SwerveDriveKinematics
                Constants.Drive.HOLONOMIC_CONTROLLER_PID_XY_CONSTRAINTS, // PID constants to correct for translation error (used to create the X and Y PID controllers)
                Constants.Drive.HOLONOMIC_CONTROLLER_PID_ROTATIONAL_CONSTRAINTS, // PID constants to correct for rotation error (used to create the rotation controller)
                Drive::setModuleStates, // Module states consumer used to output to the drive subsystem
                eventMap,
                true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                Drive // The drive subsystem. Used to properly set the requirements of path following commands
        );
    }

    private Command GetPathPlannerAutoCommand(String name)
    {
        return AutoBuilder.fullAuto(PathPlanner.loadPathGroup(name, new PathConstraints(4, 3)));
    }

    private void ConfigureAutoCommands()
    {
        AutoChooser.setDefaultOption("No auto", new WaitUntilCommand(0));

//        AutoChooser.addOption("Testpath", new FollowTrajectory(Drive, "Testpath", true));
//        AutoChooser.addOption("CurveTestPath", new FollowTrajectory(Drive, "CurveTestPath", true));
//        AutoChooser.addOption("AutoBalancePath", new AutoBalancePath(Drive));
//        AutoChooser.addOption("Coop And Engage", GetPathPlannerAutoCommand("Coop&Engage"));
//        AutoChooser.addOption("AutoBalance", Drive.Command_AutoBalance());
        AutoChooser.addOption("No Auto", new WaitCommand(0));
        AutoChooser.addOption("SimplePathStationSideEngage", GetPathPlannerAutoCommand("SimplePathStationSideEngage"));
        AutoChooser.addOption("SimplePathStationSide", GetPathPlannerAutoCommand("SimplePathStationSide"));
        AutoChooser.addOption("SimpleEngage", Drive.Command_AutoBalance());
        SmartDashboard.putData(AutoChooser);
    }

}


