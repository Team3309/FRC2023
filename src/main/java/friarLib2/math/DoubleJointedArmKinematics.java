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



    

    
}
