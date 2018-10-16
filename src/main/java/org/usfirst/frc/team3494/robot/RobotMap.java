package org.usfirst.frc.team3494.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    // For example to map the left and right motors, you could define the
    // following variables to use with your driveTrain subsystem.
    // public static int leftMotor = 1;
    // public static int rightMotor = 2;

    // If you are using multiple modules, make sure to define both the port
    // number and the module. For example you with a rangefinder:
    // public static int rangefinderPort = 1;
    // public static int rangefinderModule = 1;
    public static final int DRIVE_RIGHT_MASTER = 1;
    public static final int DRIVE_RIGHT_FOLLOW_ONE = 17;
    public static final int DRIVE_RIGHT_FOLLOW_TWO = 2;

    public static final int DRIVE_LEFT_MASTER = 14;
    public static final int DRIVE_LEFT_FOLLOW_ONE = 13;
    public static final int DRIVE_LEFT_FOLLOW_TWO = 15;

    public static final double PATH_MAX_SPEED = 3.048;

    public static final int USONIC_PIN = 0;

    public static final int ROLLER_LEFT = 4;
    public static final int ROLLER_RIGHT = 5;
    public static final int ROLLER_PISTON = 4;
    public static final int ROLLER_WINCH = 10;

    public static final int JOYSTICK_LEFT = 0;
    public static final int JOYSTICK_RIGHT = 1;
    public static final int XBOX_CONTROLLER = 2;

    public static final int LIGHT_VOLTAGE = 0;

    public static final int RIGHT_RAMP_FORWARD = 0;
    public static final int RIGHT_RAMP_REVERSE = 1;
    public static final int LEFT_RAMP_FORWARD = 2;
    public static final int LEFT_RAMP_REVERSE = 3;
    public static final int RAMP_WINCH = 11;

    public static final int RAMP_CLAW_FORWARD = 5;
    public static final int RAMP_CLAW_REVERSE = 6;

    public static final int LIFT_MOTOR = 6;
    public static final int LIFT_HALL_TOP = 0;
    public static final int LIFT_HALL_BOT = 1;

    private static final double ENCODER_PULSES_PER_REVOLUTION = 256.0D;
    public static final double WHEEL_TURNS_PER_ENCODER_TURN = 1.0 / 2.975;
    public static final double INCHES_PER_WHEEL_TURN = 6.0 * Math.PI;

    public static final double INCHES_PER_COUNT = (1.0 / ENCODER_PULSES_PER_REVOLUTION) * WHEEL_TURNS_PER_ENCODER_TURN * INCHES_PER_WHEEL_TURN;
    public static final double COUNTS_PER_INCH = 1.0 / INCHES_PER_COUNT;

    public static final double FEET_PER_COUNT = (1.0 / 12.0) * INCHES_PER_COUNT;
    public static final double COUNTS_PER_FOOT = COUNTS_PER_INCH * 12.0;

    public static final double COUNTS_PER_METER = COUNTS_PER_INCH * (1 / 2.54) * 100.0;
    public static final double METERS_PER_COUNT = 1.0 / COUNTS_PER_METER;

    public static final double INCHES_PER_EDGE = .0031;
    public static final double EDGES_PER_INCH = 1 / INCHES_PER_EDGE;

    public static final double FUDGE_FACTOR = 0.84;
    public static class Field
    {
        public static final double
                DISTANCE_TO_AUTOLINE = 6.08 ,  // in feet
                DISTANCE_TO_SWITCH = 11.67; // in feet
    }
    // private static final double ENCODER_TURNS_PER_WHEEL_TURN = 2.975;
    // public static final double COUNTS_PER_METER = 256 * ENCODER_TURNS_PER_WHEEL_TURN * (1 / (6 * Math.PI)) * (1 / 2.54) * 100;
    // public static final double COUNTS_PER_FOOT = 256 * ENCODER_TURNS_PER_WHEEL_TURN * (1 / (6 * Math.PI)) * 12;
}
