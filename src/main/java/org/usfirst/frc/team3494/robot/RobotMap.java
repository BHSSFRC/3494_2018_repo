package org.usfirst.frc.team3494.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    public static final int ROLLER_LEFT = 7;
    public static final int ROLLER_RIGHT = 8;
    // For example to map the left and right motors, you could define the
    // following variables to use with your driveTrain subsystem.
    // public static int leftMotor = 1;
    // public static int rightMotor = 2;

    // If you are using multiple modules, make sure to define both the port
    // number and the module. For example you with a rangefinder:
    // public static int rangefinderPort = 1;
    // public static int rangefinderModule = 1;
    public static final int DRIVE_LEFT_MASTER = 17;
    public static final int DRIVE_LEFT_FOLLOW_ONE = 1;
    public static final int DRIVE_LEFT_FOLLOW_TWO = 2;

    public static final int DRIVE_RIGHT_MASTER = 13;
    public static final int DRIVE_RIGHT_FOLLOW_ONE = 14;
    public static final int DRIVE_RIGHT_FOLLOW_TWO = 15;

    public static final double DRIVE_TOLERANCE = 0.01;

    public static final double PATH_MAX_SPEED = 15;

    public static final int ENCODER_LEFT_A = 9;
    public static final int ENCODER_LEFT_B = 8;
    public static final int ENCODER_RIGHT_A = 7;
    public static final int ENCODER_RIGHT_B = 6;

    public static final int JOYSTICK_LEFT = 0;
    public static final int JOYSTICK_RIGHT = 1;

    public static final int LIGHT_VOLTAGE = 0;
}
