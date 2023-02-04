// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.drive.DriveTeleop;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.auto.autos.Testpath;
import frc.robot.commands.drive.FollowTrajectory;


/**
* This class is where the bulk of the robot should be declared. Since Command-based is a
* "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
* periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
* subsystems, commands, and trigger mappings) should be declared here.
*/
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final ArmSubsystem arm = new ArmSubsystem();
    private final DriveSubsystem drive = new DriveSubsystem();
    
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // sends the autos to the dashboard
        autoChooser.setDefaultOption("No auto", new WaitUntilCommand(0));
        autoChooser.addOption("Testpath",(Command) new FollowTrajectory(drive, "Testpath", true));
        autoChooser.addOption("CurveTestPath",new FollowTrajectory(drive, "CurveTestPath", true));
        SmartDashboard.putData(autoChooser);
        // Configure the trigger bindings
        configureBindings();
        setDefaultCommands();
    }

    /**
    * Use this method to define your trigger->command mappings. Triggers can be created via the
    * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
    * predicate, or via the named factories in {@link
    * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
    * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
    * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
    * joysticks}.
    */  
    private void configureBindings() {
        //Re-zeros the gyro 
        new Trigger(OI.leftStick::getTop).whileTrue(new InstantCommand(IMU::zeroIMU));
    }
    
    private void setDefaultCommands() {
        drive.setDefaultCommand(new DriveTeleop(drive));
    } 
    
    /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return autoChooser.getSelected();
    }
}
                