import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.IMU;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants;
import java.lang.Math;


public class AutoBalance extends CommandBase {
    
    private DriveSubsystem drive;
    
    private double errorUntilFlat; //error until the robot is flat
    private double currentAngle;
    private double robotSpeed;


    public AutoBalance(DriveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    public double GetError() {
        double currentAngle = IMU.getRobotRoll().getDegrees(); //TODO Change this to pitch when IMU is oriented correctly

        return 0 - currentAngle;  
    }
    
    @Override
    public void initialize() {
        errorUntilFlat = GetError();
    }

    @Override
    public void execute() {
        int samples = 15; //smaller means average moves faster
        double error = GetError();
        errorUntilFlat = (errorUntilFlat * (samples - 1) + error) / samples;
        robotSpeed = -Math.min(Constants.Drive.CHARGE_STATION_DRIVE_KP * errorUntilFlat, 1);

        double rawSpeed = robotSpeed;

        // // for when the robot needs to drive in reverse
        // if (robotSpeed < 0) {
        // robotSpeed *= 1.35;
        // } 

        //Limit the max speed of the robot
        if (Math.abs(robotSpeed) > .25) {
            robotSpeed = Math.copySign(.25, robotSpeed);
        }

        drive.setChassisSpeeds(new ChassisSpeeds(robotSpeed, 0, 0));

        String output = String.format("Speed: %.2f %.2f \t Error: %.2f %.2f \t Angle: %.2f", rawSpeed, robotSpeed, error, errorUntilFlat, currentAngle);
        System.out.println(output);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stopChassis();
        System.out.println("Finished");
    }

    @Override
    public boolean isFinished() {

        return Math.abs(errorUntilFlat) < Constants.Drive.CHARGE_STATION_ANGLE_TRESHOLD_DEGREES;
    }
}
