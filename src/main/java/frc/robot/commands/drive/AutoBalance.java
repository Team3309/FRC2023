package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.IMU;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalance extends CommandBase {
    
    private DriveSubsystem drive;
    
    private double error;
    private double currentAngle;


    public AutoBalance() {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
     this.currentAngle = IMU.getRobotPitch().getDegrees();

     
    }

    
}
