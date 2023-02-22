package frc.robot.commands.auto.autos;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import frc.robot.commands.drive.AutoBalance;
import frc.robot.commands.drive.DriveTeleop;
import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.subsystems.DriveSubsystem;
import friarLib2.commands.RunForTime;
import friarLib2.commands.TimedInstantCommand;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;

// :(
public class AutoBalancePath extends SequentialCommandGroup {

    public AutoBalancePath(DriveSubsystem drive) {
        addRequirements(drive);
        addCommands(
            new FollowTrajectory(drive, "AutoBalancePath", true)
        );

        PathPlannerTrajectory AutoBalancePath = PathPlanner.loadPath("AutoBalancePath", 1.0, 1.0);

        addRequirements(drive);

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Balance", new AutoBalance(drive));


        
        // addCommands(
        //     new ParallelCommandGroup(
        //         new SequentialCommandGroup(
        //             new WaitCommand(2.4),
        //             new AutoBalance(drive)
        //         )
        //     )
        // );                
    }
}