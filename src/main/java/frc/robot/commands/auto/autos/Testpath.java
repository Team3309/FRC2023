package frc.robot.commands.auto.autos;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import friarLib2.commands.RunForTime;
import friarLib2.commands.TimedInstantCommand;
    
public class Testpath extends SequentialCommandGroup {

    private Timer timer = new Timer();

    public Testpath(DriveSubsystem drive) {

        timer.reset();
        timer.start();

        addCommands(
            new TimedInstantCommand(
                2,
                () -> drive.setChassisSpeeds(new ChassisSpeeds(2, 0, 0)),
                drive
            ),
            new InstantCommand(drive::stopChassis, drive)
        );
    }
    
}