/*Doesn't need changes unless we want a different controler setup. */
package frc.robot;

import friarLib2.hid.JoystickClusterGroup;
import friarLib2.hid.Joystick3309;
import friarLib2.hid.XboxController3309;

/**
 * Contains static references to controllers and joysticks
 */
public class OI
{
    // -- Driver
    public static Joystick3309 leftStick = new Joystick3309(0, Constants.JOYSTICK_DEADBAND);
    public static Joystick3309 rightStick = new Joystick3309(1, Constants.JOYSTICK_DEADBAND);

    // -- Operator
    public static XboxController3309 operatorController = new XboxController3309(2, Constants.XBOX_DEADBAND);
}