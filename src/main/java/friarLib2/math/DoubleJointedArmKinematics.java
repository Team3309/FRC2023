package friarLib2.math;

import java.lang.Math;

import javax.swing.text.AbstractDocument.LeafElement;

public class DoubleJointedArmKinematics { //TODO this probably doesn't even work

    private int LengthA; //Length of Segement A
    private int LengthB; //Length of Segement B
    private double AngleA; //Angle of Segement A
    private double AngleB; //Angle of Segement B

    public void forwardKinematics() { //TODO return calculations

        double segmentAXPosition = LengthA * Math.cos(AngleA);
        double segmentAYPosition = LengthA * Math.sin(AngleA);
        double segmentBXPosition = segmentAXPosition + LengthB * Math.cos(AngleA + AngleB);
        double segmentBYPosition = LengthB * Math.sin(AngleB) + LengthB * Math.sin(AngleA + AngleB);

    }

    public void inverseKinematics(double goalX, double goalY) {

        double hypotenuse = Math.sqrt(goalX * goalX + goalY * goalY);
        if (LengthA + LengthB < hypotenuse || Math.abs(LengthA - LengthB) > hypotenuse) {
            return;
        }
        double alpha = Math.acos((goalX * goalX + goalY * goalY + LengthA * LengthA - LengthB * LengthB)/(2 * LengthA * hypotenuse));
        double beta = Math.acos((LengthA * LengthA + LengthB * LengthB - goalX * goalX - goalY * goalY)/(2 * LengthA * LengthB)); 
        double gamma = Math.atan2(goalY, goalX);

        double Solution1A = gamma - alpha;
        double Solution1B = Math.PI - beta;
        double Solution2A = gamma + alpha;
        double Solution2B = beta - Math.PI;
    }

    public void feedForwardControl() {
        double MassA = 0;
        double MassB = 0;
        double RA = 0;
        double RB = 0;
        double IA = 0;
        double IB = 0; 

        double[] MatrixM = new double[4];
        MatrixM[0] = MassA * RA * RA + MassB * (LengthA * LengthA + RB * RB) + IA + IB + 2 * MassB * LengthA * RB * Math.cos(AngleB);
        MatrixM[1] = MassB * RB * RB + IB + MassB * LengthA * RB * Math.cos(AngleB);
        MatrixM[2] = MassB * RB * RB + IB + MassB * LengthA * RB * Math.cos(AngleB);
        MatrixM[3] = MassB * RB * RB + IB;

        double[] MatrixC = new double[4];
        MatrixC[0] = -MassB * LengthA * RB * AngleB;
        MatrixC[1] = -MassB * LengthA * RB * Math.sin(AngleB) * (AngleA + AngleB);
        MatrixC[2] = MassB * LengthA * RB * Math.sin(AngleB) * AngleA;
        MatrixC[3] = 0;

        double Ta = 9.8 * Math.cos(AngleA) * (MassA * RA + MassB * LengthA) + MassB * RB * 9.8 * Math.cos(AngleA + AngleB);
        double Tb = MassB * RB * 9.8 * Math.cos(AngleA + AngleB);

        double[] MatrixTg = new double[2];
        MatrixTg[0] = Ta;
        MatrixTg[1] = Tb;

        // double T = MatrixM + MatrixC + MatrixTg;

        // double[] MatrixTg = new double[4];
        // MatrixTg[0] = ;


    }





    

    
}
