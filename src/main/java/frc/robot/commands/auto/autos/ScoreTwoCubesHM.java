package frc.robot.commands.auto.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ArmSubsystem;

import java.util.ArrayList;
import java.util.HashMap;

public class ScoreTwoCubesHM extends SequentialCommandGroup {



    public ScoreTwoCubesHM(DriveSubsystem drive, ArmSubsystem arm, PathPlannerTrajectory trajectory) {
        addRequirements(drive);
        addRequirements(arm);

        ArrayList<PathPlannerTrajectory> pathGroup1 = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("ScoreTwoCubes", new PathConstraints(4, 3));
        PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, DriverStation.getAlliance()); // Mirror if we're on the red alliance

        HashMap<String, Command> eventmap = new HashMap<>();
        eventmap.put("CloseClamp", arm.ToggleClamp());
        //eventmap.put("MoveArmToHigh", );
        eventmap.put("ScoreHigh", arm.ToggleClamp());
        //eventmap.put("MoveArmToBackToPickUp", );
        eventmap.put("PickupGamePiece", arm.ToggleClamp());
        //eventmap.put("MoveToMid", );
        eventmap.put("ScoreCubeMid", arm.ToggleClamp());

    }

}
