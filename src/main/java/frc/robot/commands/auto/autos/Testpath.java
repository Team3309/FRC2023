package frc.robot.commands.auto.autos;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;



public class Testpath extends CommandBase {

    
    public Testpath(DriveSubsystem drive)  {
	}

	PathPlannerTrajectory Testpath = PathPlanner.loadPath("Test path", new PathConstraints(4, 3));
    
}

