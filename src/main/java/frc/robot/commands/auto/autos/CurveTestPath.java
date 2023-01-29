package frc.robot.commands.auto.autos;
import java.util.ArrayList;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class CurveTestPath extends CommandBase {

    
    public CurveTestPath(DriveSubsystem drive)  {
	}

    // This will load the file "CurveTestPath" and generate it with different path constraints for each segment
    ArrayList<PathPlannerTrajectory> CurveTestPaths = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup(
    "Curve Test Path", 
    new PathConstraints(4, 3), 
    new PathConstraints(2, 2), 
    new PathConstraints(3, 3));
}
