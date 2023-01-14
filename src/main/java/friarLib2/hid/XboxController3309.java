package friarLib2.hid;

import edu.wpi.first.wpilibj.XboxController;
import friarLib2.utility.Vector3309;

/**
 * Represents an Xbox controller, but with built in deadband calculation
 */
public class XboxController3309 extends XboxController {

    public double deadband;

    public XboxController3309 (final int port, double deadband) {
        super(port);

        this.deadband = deadband;
    }

    public double getLeftXWithDeadband () {
        return applyDeadband(super.getLeftX(), deadband);
    }

    public double getRightXWithDeadband () {
        return applyDeadband(super.getRightX(), deadband);
    }

    public double getLeftYWithDeadband () {
        return applyDeadband(super.getLeftY(), deadband);
    }

    public double getRightYWithDeadband () {
        return applyDeadband(super.getRightY(), deadband);
    }

    public double getLeftTriggerAxisWithDeadband () {
        return applyDeadband(super.getLeftTriggerAxis(), deadband);
    }

    public double getRightTriggerAxisWithDeadband () {
        return applyDeadband(super.getRightTriggerAxis(), deadband);
    }

    /**
     * @return A vector that represents the position of the left joystick,
     * without deadband
     */
    public Vector3309 getLeftVector () {
        return Vector3309.fromCartesianCoords(getLeftX(), -getLeftY());
    }

    /**
     * @return A vector that represents the position of the left joystick,
     * with deadband
     */
    public Vector3309 getLeftVectorWithDeadband () {
        return Vector3309.fromCartesianCoords(getLeftXWithDeadband(), -getLeftYWithDeadband());
    }

    /**
     * @return A vector that represents the position of the right joystick,
     * without deadband
     */
    public Vector3309 getRightVector () {
        return Vector3309.fromCartesianCoords(getRightX(), -getRightY());
    }

    /**
     * @return A vector that represents the position of the right joystick,
     * with deadband
     */
    public Vector3309 getRightVectorWithDeadband () {
        return Vector3309.fromCartesianCoords(getRightXWithDeadband(), -getRightYWithDeadband());
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
        return (Math.abs(joystickValue) > deadband) ? joystickValue : 0;
    }
}
