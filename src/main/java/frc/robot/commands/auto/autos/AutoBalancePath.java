package frc.robot.commands.auto.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.subsystems.DriveSubsystem;

import java.util.HashMap;

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
        //eventMap.put("Balance", new AutoBalance(drive));


        
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