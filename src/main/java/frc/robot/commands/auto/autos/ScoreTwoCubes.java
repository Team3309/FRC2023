package frc.robot.commands.auto.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.subsystems.DriveSubsystem;

import java.util.ArrayList;
import java.util.HashMap;

public class ScoreTwoCubes extends SequentialCommandGroup {

    public ScoreTwoCubes(DriveSubsystem drive) {
        addRequirements(drive);

        ArrayList<PathPlannerTrajectory> pathGroup1 = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("ScoreTwoCubes", new PathConstraints(4, 3));

        HashMap<String, Command> eventMap = new HashMap<String, Command>();

    }

}
