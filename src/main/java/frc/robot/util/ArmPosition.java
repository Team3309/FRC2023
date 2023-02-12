package frc.robot.util;

import org.ejml.ops.DOperatorBinary;

import frc.robot.Constants.Arm;

public class ArmPosition {
    // :D
    private final double xPosition;
    private final double yPosition;
    private final double angleA;
    private final double angleB;

    private ArmPosition(double x, double y, double a, double b) {
        xPosition = x;
        yPosition = y;
        angleA = a;
        angleB = b;
    }
    
    public static ArmPosition fromCoords(double x, double y) {
        // TODO: Kinematics

        return new ArmPosition(x, y, 0, 0);
    }

    public static ArmPosition fromAngles(double a, double b) {
        // TODO: Kinematics

        return new ArmPosition(0, 0, a, b);
    }
}
