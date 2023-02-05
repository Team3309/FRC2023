package friarLib2.math;

import java.lang.Math;

public class DoubleJointedArmKinematics { //TODO this probably doesn't even work
    private double SegmentA; //connected to Shoulder joint
    private double SegmentB; //connected to end effector
    private int LengthA; //Length of Segement A
    private int LengthB; //Length of Segement B
    private double AngleA; //Angle of Segement A
    private double AngleB; //Angle of Segement B

    public double[][] PositionOfSA() {
        double SX = (Double) null;
        double SY = (Double) null;
        //Initialize the matrix
        double[][] matrix1 = new double [(int) SX][(int) SY];
        double[][] matrix2 = new double [(int) (LengthA * Math.cos(AngleA))][(int) (LengthA * Math.sin(AngleA))];

        matrix1 = matrix2;

        return matrix1;

    }

    

    
}
