/*Shouldn't need major changes */
package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.IMU;
import frc.robot.Swerve.SwerveModule3309;
import friarLib2.hardware.SwerveModule;

import java.util.concurrent.atomic.AtomicReference;

import static frc.robot.Constants.Drive.*;

public class DriveSubsystem extends SubsystemBase {

    private final Field2d field = new Field2d();

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private final SwerveDriveOdometry swerveOdometry;
    private final SwerveDriveKinematics swerveKinematics;
    private Pose2d currentRobotPose = new Pose2d();

    /**
     * Initialize the swerve modules, imu, and Kinematics/Odometry objects
     */
    public DriveSubsystem()
    {
        frontLeftModule = new SwerveModule3309(225, FRONT_LEFT_MODULE_IDS, "Front left");
        frontRightModule = new SwerveModule3309(315, FRONT_RIGHT_MODULE_IDS, "Front right");
        backLeftModule = new SwerveModule3309(135, BACK_LEFT_MODULE_IDS, "Back left");
        backRightModule = new SwerveModule3309(45, BACK_RIGHT_MODULE_IDS, "Back right");

        swerveKinematics = new SwerveDriveKinematics(
            FRONT_LEFT_MODULE_TRANSLATION,
            FRONT_RIGHT_MODULE_TRANSLATION,
            BACK_LEFT_MODULE_TRANSLATION,
            BACK_RIGHT_MODULE_TRANSLATION
        );

        swerveOdometry = new SwerveDriveOdometry(
            swerveKinematics,
            IMU.getRobotYaw(),
            new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
            }
        );

        IMU.zeroIMU();

        SmartDashboard.putData("Calibrate Swerve", Command_CalibrateSwerve());
        SmartDashboard.putData("Odometry", field);
    }

    public void setModuleStates (SwerveModuleState[] states) {
        frontLeftModule.setState(states[0]);
        frontRightModule.setState(states[1]);
        backLeftModule.setState(states[2]);
        backRightModule.setState(states[3]);
    }

    /**
     * Use the CANCoders to rezero each swerve module
     */
    public void zeroModules () {
        frontLeftModule.zeroSteering();
        frontRightModule.zeroSteering();
        backLeftModule.zeroSteering();
        backRightModule.zeroSteering();
    }

    /**
     * Calculate and set the required SwerveModuleStates for a given ChassisSpeeds
     * 
     * @param speeds
     */
    public void setChassisSpeeds (ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = swerveKinematics.toSwerveModuleStates(speeds); //Generate the swerve module states
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveModule3309.ABSOLUTE_MAX_DRIVE_SPEED);
        setModuleStates(moduleStates);
    }

    /**
     * Set the target speeds to zero (stop the drivetrain)
     */
    public void stopChassis () {
        setChassisSpeeds(new ChassisSpeeds());
    }

    /**
     * Get the current robot pose according to dead-reckoning odometry
     * See https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-odometry.html for more details
     * 
     * @return Current robot pose
     */
    public Pose2d GetPose() {
        return currentRobotPose;
    }
    public SwerveDriveKinematics GetKinematics() { return swerveKinematics; }

    /**
     * Resets the odometry without resetting the IMU
     */
    public void ResetOdometry()
    {
        swerveOdometry.resetPosition(
            IMU.getRobotYaw(),
            new SwerveModulePosition[] {
                    frontLeftModule.getPosition(),
                    frontRightModule.getPosition(),
                    backLeftModule.getPosition(),
                    backRightModule.getPosition()
            },
            new Pose2d()
        );
    }

    /**
     * Set the odometry, swerve modules, and IMU readings
     * 
     * @param pose Pose to be written to odometry
     */
    public void HardResetOdometry(Pose2d pose) {
        IMU.tareIMU(pose.getRotation());
        frontLeftModule.zeroPosition();
        frontRightModule.zeroPosition();
        backLeftModule.zeroPosition();
        backRightModule.zeroPosition();

        swerveOdometry.resetPosition(
            new Rotation2d(),
            new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            },
            pose
        );
    }

    @Override
    public void periodic() {
        //Update the odometry using module states and chassis rotation
        currentRobotPose = swerveOdometry.update(
            IMU.getRobotYaw(),
            new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
            }
        );

//        frontLeftModule.outputToDashboard();
//        frontRightModule.outputToDashboard();
//        backLeftModule.outputToDashboard();
//        backRightModule.outputToDashboard();

        field.setRobotPose(currentRobotPose);

//        SmartDashboard.putNumber("Robot heading", IMU.getRobotYaw().getDegrees());
//        SmartDashboard.putNumber("Meters to target", LimelightVision.getMetersFromTarget());
    }


    // -------------------------------------------------------------------------------------------------------------------------------------
    // -- Commands
    // -------------------------------------------------------------------------------------------------------------------------------------
    public CommandBase Command_AutoBalance()
    {
        return Commands.sequence(
              Command_DriveDistance(1.75, 1.45)
            , Command_DriveDistance(0.75, 0.2)
            , Command_DriveDistance(0.5, 1.5).raceWith(Command_WaitUntilFalling())
            , Command_DriveDistance(-0.5, 0.4)
        );
    }

    public CommandBase Command_DriveStraight(double metersPerSecond)
    {
        return run( () -> setChassisSpeeds(new ChassisSpeeds(metersPerSecond, 0, 0)) )
                .beforeStarting(new PrintCommand(System.currentTimeMillis() + "  Drive Straight " + metersPerSecond));
    }

    public CommandBase Command_DriveDistance(double metersPerSecond, double distanceInMeters)
    {
        final AtomicReference<Pose2d> startingPose = new AtomicReference<>(); // wrapper to allow us to change captured object inside a lambda

        return Commands.sequence(
                  runOnce(() -> startingPose.set(currentRobotPose))
                , Command_DriveStraight(metersPerSecond)
                        .until(() -> {
                            if (metersPerSecond > 0)
                            {
                                return currentRobotPose.getX() > startingPose.get().getX() + distanceInMeters;
                            }
                            return currentRobotPose.getX() < startingPose.get().getX() - distanceInMeters;
                        })
        );
    }

    // -------------------------------------------------------------------------------------------------------------------------------------
    // -- Internal Commands
    // -------------------------------------------------------------------------------------------------------------------------------------
    private CommandBase Command_WaitUntilFalling()
    {
        final int numSamples = 8;
        AtomicReference<LinearFilter> filter = new AtomicReference<>();

        return Commands.sequence(
                // -- Create a new filter and initialize the values to the current angle
                Commands.runOnce( () ->
                {
                    double angle = IMU.getRobotPitch().getDegrees();
                    filter.set(LinearFilter.movingAverage(numSamples));
                    for (int i = 0; i < numSamples; i++)
                    {
                        filter.get().calculate(angle);
                    }
                })
                // -- Use the filter to figure out when we start falling for real
                , Commands.waitUntil(() ->
                {
                    double current = IMU.getRobotPitch().getDegrees();
                    double filtered = filter.get().calculate(current);
                    //System.out.printf("%d %.2f  |  %.2f%n", System.currentTimeMillis(), current, filtered);
                    return filtered < CHARGE_STATION_TILT_ANGLE;
                })
        );
    }

    private CommandBase Command_CalibrateSwerve()
    {
        return runOnce(() ->
        {
            CANCoder[] canCoders = { frontLeftModule.GetCanCoder(), frontRightModule.GetCanCoder(), backLeftModule.GetCanCoder(), backRightModule.GetCanCoder() };

            for (CANCoder canCoder : canCoders)
            {
                int id = canCoder.getDeviceID();

                canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
                canCoder.configMagnetOffset(0);

                double absolutePosition = canCoder.getAbsolutePosition();
                int canCoderSlotValue = (int) (absolutePosition * 100 + 0.5);

                System.out.println(
                        "Setting custom parameter on encoder " + id +
                                " to " + canCoderSlotValue +
                                " (absolute position = " + absolutePosition + " degrees)"
                );

                ErrorCode e = canCoder.configSetCustomParam(canCoderSlotValue, 0);
                if (e != ErrorCode.OK) {
                    System.out.println("Error setting custom parameter on encoder " + id);
                }
            }
        });
    }
}
