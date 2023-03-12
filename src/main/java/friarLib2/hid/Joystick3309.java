package friarLib2.hid;

import edu.wpi.first.wpilibj.Joystick;
import friarLib2.utility.Vector3309;

/**
 * Represents a joystick, but with built in deadband calculation
 */
public class Joystick3309 extends Joystick {

    private double deadband;

    public Joystick3309 (final int port, double deadband) {
        super(port);

        this.deadband = deadband;
    }

    public double getXWithDeadband () {
        return applyDeadband(super.getX(), deadband);
    }

    public double getYWithDeadband () {
        return applyDeadband(super.getY(), deadband);
    }

    /**
     * @return A vector that represents the position of the joystick,
     * without deadband
     */
    public Vector3309 getVector () {
        return Vector3309.fromCartesianCoords(getX(), -getY());
    }

    /**
     * @return A vector that represents the position of the joystick,
     * with deadband
     */
    public Vector3309 getVectorWithDeadband () {
        return Vector3309.fromCartesianCoords(getXWithDeadband(), -getYWithDeadband());
    }

    /**
     * Return zero if the absolute value of joystickValue is less than deadband, else, return joystickValue.
     * Ueseful for ensuring a zero value when a joystick is released.
     * 
     * @param joystickValue the value of the joystick axis
     * @param deadband the deadband to apply
     * @return the adjusted joystickValue
     */
    public static double applyDeadband (double joystickValue, double deadband) {
        return (Math.abs(joystickValue) > deadband) ? applyLerp(joystickValue, deadband) : 0;
    }

    private static double applyLerp (double joystickValue, double deadband) {
        double result = deadband - deadband*(Math.abs(joystickValue)) - (Math.abs(joystickValue));
        return (joystickValue > 0 ? -result : result);
    }
}
