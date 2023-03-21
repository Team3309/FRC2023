package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class EstimatingDistance {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("Limelight");
    NetworkTableEntry ty = table.getEntry("ty");
    double targettOffsetAngle_Vertical = ty.getDouble(0.0);

    double limelightMountAngleDegrees = -25.0;

    double limelightLensHeightInches = 49.865933383;
    double goalHeightInches = 43.875;

    // Basic maths
    double angleToGoalDegrees = limelightMountAngleDegrees + targettOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180);

    // calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);

}
