package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import friarLib2.vision.LimelightCamera;
import friarLib2.vision.IVisionCamera;
import edu.wpi.first.math.*;

public class EstimatingDistance {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("Limelight");
    NetworkTableEntry ty = table.getEntry("ty");



}
