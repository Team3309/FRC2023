// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Swerve.SwerveCANIDs;
import friarLib2.utility.PIDParameters;

/**
* The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
* constants. This class should not be used for any other purpose. All constants should be declared
* globally (i.e. public static). Do not put anything functional in this class.
*
* <p>It is advised to statically import this class (or one of its inner classes) wherever the
* constants are needed, to reduce verbosity.
*/
public final class Constants { 
    
    
    /**
    *  For Constants that are not in a subsystem
    */
    
    public static final double JOYSTICK_DEADBAND = 0.09; // TODO a lot of the constants in this file are placeholders, remainder to change them
    public static final double XBOX_DEADBAND = 0.05;
    
    public static final int PCM_CAN_ID = 1;
    public static final int PIGEON_IMU_ID = 19;
    
    public static final PneumaticsModuleType PCM_TYPE= PneumaticsModuleType.REVPH;
    
    
    /**
    * Constants for the Drivetrain
    */
    public static class Drive {
        /********** CAN ID's **********/
        public static final SwerveCANIDs FRONT_LEFT_MODULE_IDS = new SwerveCANIDs(4, 6, 52);
        public static final SwerveCANIDs FRONT_RIGHT_MODULE_IDS = new SwerveCANIDs(3, 5, 53);
        public static final SwerveCANIDs BACK_LEFT_MODULE_IDS = new SwerveCANIDs(1, 2, 50);
        public static final SwerveCANIDs BACK_RIGHT_MODULE_IDS = new SwerveCANIDs(8, 7, 51);
        
        /********** Module Translations **********/
        public static final Translation2d FRONT_LEFT_MODULE_TRANSLATION = new Translation2d(0.34671, 0.23241);
        public static final Translation2d FRONT_RIGHT_MODULE_TRANSLATION = new Translation2d(0.34671, -0.23241);
        public static final Translation2d BACK_LEFT_MODULE_TRANSLATION = new Translation2d(-0.34671, 0.23241);
        public static final Translation2d BACK_RIGHT_MODULE_TRANSLATION = new Translation2d(-0.34671, -0.23241);
        
        /********** Autonomous Motion Envelope **********/
        public static final double MAX_AUTON_SPEED = 2; // Meters/second
        public static final double MAX_AUTON_ACCELERATION = 2.5; // Meters/second squared
        public static final double MAX_AUTON_ANGULAR_SPEED = 400; // Degrees/second
        public static final double MAX_AUTON_ANGULAR_ACCELERATION = 200; // Degrees/second squared
        
        /********** Holonomic Controller Gains **********/
        public static final PIDController HOLONOMIC_CONTROLLER_PID_X = new PIDController(4, .75,0.5); //9, 3, 0
        public static final PIDController HOLONOMIC_CONTROLLER_PID_Y = new PIDController(4, .75,0.5);
        public static final ProfiledPIDController HOLONOMIC_CONTROLLER_PID_THETA = new ProfiledPIDController(5, 0, 0, new TrapezoidProfile.Constraints(MAX_AUTON_ANGULAR_SPEED, MAX_AUTON_ANGULAR_ACCELERATION));
        
        /******** PID Gains ********/
        public static final PIDController VISION_AIM_PID = new PIDController(0.3, 0, 0);
        
        /********** Teleop Control Adjustment **********/
        public static final double MAX_TELEOP_SPEED = 6; // Meters/second
        public static final double MAX_TELEOP_ROTATIONAL_SPEED = Math.toRadians(700); // Radians/second
        public static final double MAX_TELEOP_ACCELERATION = 7; // Maters/second squared
        public static final double MAX_TELEOP_DECELERATION = 11;
        
    }
    
    /**
     * Constants for the Arm
     */
    public static class Arm {
        /********** CAN ID's **********/
        public static final int JOINT_A_MOTOR_ID = 9;
        public static final int JOINT_B_MOTOR_ID = 10;
        
        /******** PID Gains ********/
        public static final PIDParameters MOTOR_A_PID_GAINS = new PIDParameters(0, 0, 0, "Joint A Motor PID");
        public static final PIDParameters MOTOR_B_PID_GAINS = new PIDParameters(0, 0, 0, "Joint B Motor PID");
        
        /******** Gear Ratios ********/
        public static final double JOINT_A_MOTOR_GEAR_RATIO = 1.0;
        public static final double JOINT_B_MOTOR_GEAR_RATIO = 1.0;
    }

    /*
     * Constants for the clamp
     */
    public static class Clamp {
        /********** Solenoid PCM Ports **********/
        public static final int CLAMP_SOLENOID_ID = 1;
    }

    /* 
     * Constants for the Intake
    */
    public static class Intake {
        /********** CAN ID's **********/
        public static final int TOP_INTAKE_MOTOR_ID = 4;
        public static final int BOTTOM_INTAKE_MOTOR_ID = 3;

        /********** Tuning Constants **********/
        public static final double TOP_INTAKE_MOTOR_POWER = 1;
        public static final double BOTTOM_INTAKE_MOTOR_POWER = 2;
    }

    /*
     * Constants for the Turntable
     */
    public static class Turntable {
        /********** CAN ID's **********/
        public static final int TURNTABLE_MOTOR_ID = 5;

        /********** Tuning Constants **********/
        public static final double TURNTABLE_MOTOR_POWER = 1;
        public static final double TURNTABLE_DEFAULT_DEGREES = 0;

        /********** Rotating Constnats **********/
        public static final double TURNATABLE_DEFAULT_POSITION = 0;
        public static final double TURNTABLE_ORIENTATION = 1;
       
    }
}



