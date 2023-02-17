package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.IMU;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants;
import java.lang.Math;

public class AutoBalance extends CommandBase {
    
    private DriveSubsystem drive;
    
    private double errorUntilFlat; //error untill the robot is flat
    private double currentAngle;
    private double robotSpeed;


    public AutoBalance(DriveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        this.currentAngle = IMU.getRobotPitch().getDegrees();

        errorUntilFlat = Constants.Drive.CHARGE_STATION_GOAL_DEGREES - currentAngle;
        robotSpeed = -Math.min(Constants.Drive.CHARGE_STATION_DRIVE_KP * errorUntilFlat, 1);

        // for when the robot needs to drive in reverse
        if (robotSpeed < 0) {
        robotSpeed *= 1.35;
        } 

        //Limit the max speed of the robot
        if (Math.abs(robotSpeed) > 0.4) {
        robotSpeed = Math.copySign(0.4, robotSpeed);
        }

        drive.setChassisSpeeds(new ChassisSpeeds(robotSpeed, 0, 0));   
        
        System.out.println("Auto balancing speed: " + robotSpeed);
        System.out.println("Error: " + errorUntilFlat);
        System.out.println("Current angle " + currentAngle);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stopChassis();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(errorUntilFlat) < Constants.Drive.CHARGE_STATION_ANGLE_TRESHOLD_DEGREES;
    }
}
