/*Doesn't need changes unless we want a different controler setup. */
package frc.robot;

import friarLib2.hid.JoystickClusterGroup;
import friarLib2.hid.Joystick3309;
import friarLib2.hid.XboxController3309;

/**
 * Contains static references to controllers and joysticks
 */
public class OI {
    // Joysticks
    public static Joystick3309 leftStick = new Joystick3309(0, Constants.JOYSTICK_DEADBAND);
    public static Joystick3309 rightStick = new Joystick3309(1, Constants.JOYSTICK_DEADBAND);

    public static JoystickClusterGroup leftStickLeftCluster = new JoystickClusterGroup(leftStick, JoystickClusterGroup.Side.left);
    public static JoystickClusterGroup leftStickRightCluster = new JoystickClusterGroup(leftStick, JoystickClusterGroup.Side.right);
    public static JoystickClusterGroup rightStickLeftCluster = new JoystickClusterGroup(rightStick, JoystickClusterGroup.Side.left);
    public static JoystickClusterGroup rightStickRightCluster = new JoystickClusterGroup(rightStick, JoystickClusterGroup.Side.right);

    // Xbox controller
    public static XboxController3309 operatorController = new XboxController3309(2, Constants.XBOX_DEADBAND);
}