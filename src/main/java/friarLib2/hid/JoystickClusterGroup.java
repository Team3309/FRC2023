package friarLib2.hid;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Represents one of the six-button clusters on either side of a Joystick
 * 
 * Triggers when any of the buttons in that cluster are pressed
 */
public class JoystickClusterGroup extends Trigger {

    Joystick stick;
    Side side;

    public static final int LEFT_CLUSTER_1_ID = 11;
    public static final int LEFT_CLUSTER_2_ID = 12;
    public static final int LEFT_CLUSTER_3_ID = 13;
    public static final int LEFT_CLUSTER_4_ID = 14;
    public static final int LEFT_CLUSTER_5_ID = 15;
    public static final int LEFT_CLUSTER_6_ID = 16;
    public static final int RIGHT_CLUSTER_1_ID = 5;
    public static final int RIGHT_CLUSTER_2_ID = 6;
    public static final int RIGHT_CLUSTER_3_ID = 7;
    public static final int RIGHT_CLUSTER_4_ID = 8;
    public static final int RIGHT_CLUSTER_5_ID = 9;
    public static final int RIGHT_CLUSTER_6_ID = 10;

    public JoystickClusterGroup(Joystick stick, Side side) {
        this.stick = stick;
        this.side = side;
    }

    public boolean get() {
        if (side == Side.left) {
            return stick.getRawButton(LEFT_CLUSTER_1_ID) ||
                    stick.getRawButton(LEFT_CLUSTER_2_ID) ||
                    stick.getRawButton(LEFT_CLUSTER_3_ID) ||
                    stick.getRawButton(LEFT_CLUSTER_4_ID) ||
                    stick.getRawButton(LEFT_CLUSTER_5_ID) ||
                    stick.getRawButton(LEFT_CLUSTER_6_ID);
        } else {
            return stick.getRawButton(RIGHT_CLUSTER_1_ID) ||
                    stick.getRawButton(RIGHT_CLUSTER_2_ID) ||
                    stick.getRawButton(RIGHT_CLUSTER_3_ID) ||
                    stick.getRawButton(RIGHT_CLUSTER_4_ID) ||
                    stick.getRawButton(RIGHT_CLUSTER_5_ID) ||
                    stick.getRawButton(RIGHT_CLUSTER_6_ID);
        }
    }

    public static enum Side {
        left,
        right
    }
}