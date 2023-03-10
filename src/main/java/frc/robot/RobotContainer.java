// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
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


    private void ConfigureBindings()
    {
        // ----------------------------------------------------------------------------------------
        // -- Driver
        // ----------------------------------------------------------------------------------------

        // -- Clamp
        new Trigger(OI.rightStick::getTrigger).onTrue(Arm.ActuateClamp(true));
        new Trigger(OI.rightStick::getTop).onTrue(Arm.ActuateClamp(false));

        // -- Auto Turn
        new Trigger(OI.leftStick::getTrigger).whileTrue(new TurnInDirectionOfTarget(Drive));



        // ----------------------------------------------------------------------------------------
        // -- Operator
        // ----------------------------------------------------------------------------------------

        // -- Arm
        new Trigger(OI.operatorController::getAButton).onTrue(Arm.SetPositionCommand(ArmSubsystem.ArmPosition.Stowed));
        new Trigger(OI.operatorController::getBButton).onTrue(Arm.SetPositionCommand(ArmSubsystem.ArmPosition.ScoreHybrid));
        new Trigger(OI.operatorController::getXButton).onTrue(Arm.SetPositionCommand(ArmSubsystem.ArmPosition.ScoreMid));
        new Trigger(OI.operatorController::getYButton).onTrue(Arm.SetPositionCommand(ArmSubsystem.ArmPosition.ScoreTop));

        new Trigger(OI.operatorController::getRightBumper).onTrue(Arm.SetDirectionCommand(ArmSubsystem.ArmDirection.Forward));
        new Trigger(OI.operatorController::getLeftBumper).onTrue(Arm.SetDirectionCommand(ArmSubsystem.ArmDirection.Backward));
        
        // -- Intake
        //new Trigger(OI.XboxController::leftBumper).whileTrue(new ActivateRollers());

        // -- AutoBalance
        new Trigger(OI.rightStick::getTrigger).onTrue(Drive.AutoBalanceCommand());

        // -- Vision
        //new Trigger(OI.rightStick::getTop).onTrue(LimelightVision.SetPipelineCommand(0));
        //new Trigger(OI.rightStick::get).onTrue(ApriltagVision.SetPipelineCommand(1));

        // -- Turntable
        // new Trigger(OI.operatorController::getAButton).whileTrue(new InstantCommand(new TurntableSubsystem()::defaultPosition));

        // -- Reset Odometry
        new Trigger(OI.rightStickRightCluster::get).onTrue(Commands.runOnce(Drive::ResetOdometry));

        // -- Re-zeros the gyro
        new Trigger(OI.leftStick::getTop).whileTrue(new InstantCommand(IMU::zeroIMU));
    }


    private void SetDefaultCommands()
    {
        Drive.setDefaultCommand(new DriveTeleop(Drive));
    }


    private void ConfigureSwerveAutoBuilder()
    {
        // -- Map Path Planner events to Commands
        HashMap<String, Command> eventMap = new HashMap<>();

        eventMap.put("BalanceForward", Drive.AutoBalanceCommand());

        eventMap.put("Arm_ScoreTop_Forward", Arm.SetPositionAndDirectionCommand(ArmSubsystem.ArmPosition.ScoreTop, ArmSubsystem.ArmDirection.Forward));
        eventMap.put("Arm_ScoreTop_Backward", Arm.SetPositionAndDirectionCommand(ArmSubsystem.ArmPosition.ScoreTop, ArmSubsystem.ArmDirection.Backward));
        eventMap.put("Arm_ScoreMid_Forward", Arm.SetPositionAndDirectionCommand(ArmSubsystem.ArmPosition.ScoreMid, ArmSubsystem.ArmDirection.Forward));
        eventMap.put("Arm_ScoreMid_Backward", Arm.SetPositionAndDirectionCommand(ArmSubsystem.ArmPosition.ScoreMid, ArmSubsystem.ArmDirection.Backward));
        eventMap.put("Arm_ScoreHybrid_Forward", Arm.SetPositionAndDirectionCommand(ArmSubsystem.ArmPosition.ScoreHybrid, ArmSubsystem.ArmDirection.Forward));
        eventMap.put("Arm_PickupHybrid_Backward", Arm.SetPositionAndDirectionCommand(ArmSubsystem.ArmPosition.ScoreHybrid, ArmSubsystem.ArmDirection.Backward));
        eventMap.put("Arm_PickupFloor_Backward", Arm.SetPositionAndDirectionCommand(ArmSubsystem.ArmPosition.PickupFloor, ArmSubsystem.ArmDirection.Backward));
        

        eventMap.put("Clamp_Close", Arm.ActuateClamp(true));
        eventMap.put("Clamp_Open", Arm.ActuateClamp(false));

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

        AutoChooser.addOption("Testpath", new FollowTrajectory(Drive, "Testpath", true));
        AutoChooser.addOption("CurveTestPath", new FollowTrajectory(Drive, "CurveTestPath", true));
        AutoChooser.addOption("AutoBalancePath", new AutoBalancePath(Drive));
        AutoChooser.addOption("Coop And Engage", GetPathPlannerAutoCommand("Coop&Engage"));

        SmartDashboard.putData(AutoChooser);
    }

}


