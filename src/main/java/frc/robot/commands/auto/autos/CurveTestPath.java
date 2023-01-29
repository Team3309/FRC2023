package frc.robot.commands.auto.autos;
import java.util.ArrayList;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CurveTestPath extends SequentialCommandGroup {

    
    public CurveTestPath(DriveSubsystem drive)  {
        addCommands(
            new FollowTrajectory(drive, "CurveTestPath", true)
        );
	}

    // This will load the file "CurveTestPath" and generate it with different path constraints for each segment
    ArrayList<PathPlannerTrajectory> CurveTestPaths = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup(
    "Curve Test Path", 
    new PathConstraints(4, 3), 
    new PathConstraints(2, 2), 
    new PathConstraints(3, 3));
}
