package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.IMU;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalance extends CommandBase
{
    private DriveSubsystem drive;

    private double errorUntilFlat;
    private double currentAngle;
    private double robotSpeed;

    public AutoBalance(DriveSubsystem drive)
    {
        this.drive = drive;
        addRequirements(drive);
    }

    public double GetError()
    {
        double currentAngle = IMU.getRobotPitch().getDegrees();

        return 0 - currentAngle;
    }

    @Override
    public void initialize()
    {
        errorUntilFlat = GetError();
    }

    @Override
    public void execute()
    {
        int samples = 3; //Smaller means average moves faster
        double error = GetError();
        errorUntilFlat = (errorUntilFlat * (samples - 1) + error) / samples;
        robotSpeed = -Math.min(Constants.Drive.CHARGE_STATION_DRIVE_KP * errorUntilFlat, 0.25);

        double rawSpeed = robotSpeed;

        //Limit
        if (Math.abs(robotSpeed) > .25)
        {
            robotSpeed = Math.copySign(.25, robotSpeed);
        }

        drive.setChassisSpeeds(new ChassisSpeeds(robotSpeed, 0, 0));

        String output = String.format("Speed: %.2f %.2f \t Error: %.2f \t Angle: %.2f", rawSpeed, robotSpeed, error, errorUntilFlat, currentAngle);
        System.out.println(output);
    }

    @Override
    public void end(boolean interrupted)
    {
        drive.stopChassis();
    }

    @Override
    public boolean isFinished()
    {
        return Math.abs(errorUntilFlat) < Constants.Drive.CHARGE_STATION_ANGLE_TRESHOLD_DEGREES;
    }
}
