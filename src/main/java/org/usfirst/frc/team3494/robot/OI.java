package org.usfirst.frc.team3494.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import org.usfirst.frc.team3494.robot.commands.ramps.ExtendRamps;
import org.usfirst.frc.team3494.robot.commands.ramps.OpenClaw;
import org.usfirst.frc.team3494.robot.commands.ramps.RunWinch;
import org.usfirst.frc.team3494.robot.commands.rollerclaw.InvertClawState;
import org.usfirst.frc.team3494.robot.commands.rollerclaw.Roll;
import org.usfirst.frc.team3494.robot.commands.rollerclaw.StopRoll;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    //// CREATING BUTTONS
    // One type of button is a joystick button which is any button on a
    //// joystick.
    // You create one by telling it which joystick it's on and which button
    // number it is.
    // Joystick stick = new Joystick(port);
    // Button button = new JoystickButton(stick, buttonNumber);

    private Joystick joyLeft = new Joystick(RobotMap.JOYSTICK_LEFT);
    private Joystick joyRight = new Joystick(RobotMap.JOYSTICK_RIGHT);

    private XboxController xbox = new XboxController(RobotMap.XBOX_CONTROLLER);

    // There are a few additional built in buttons you can use. Additionally,
    // by subclassing Button you can create custom triggers and bind those to
    // commands the same as any other Button.

    //// TRIGGERING COMMANDS WITH BUTTONS
    // Once you have a button, it's trivial to bind it to a button in one of
    // three ways:

    // Start the command when the button is pressed and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenPressed(new ExampleCommand());

    // Run the command while the button is being held down and interrupt it once
    // the button is released.
    // button.whileHeld(new ExampleCommand());

    // Start the command when the button is released and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenReleased(new ExampleCommand());
    OI() {
        // primary controls
        JoystickButton leftThumb = new JoystickButton(joyLeft, 3);
        leftThumb.whenPressed(new OpenClaw());

        JoystickButton rightBase_Winch = new JoystickButton(joyRight, 11);
        rightBase_Winch.whenPressed(new RunWinch(0.5));
        rightBase_Winch.whenReleased(new RunWinch(0));

        JoystickButton rightBase_WinchOut = new JoystickButton(joyRight, 5);
        rightBase_WinchOut.whenPressed(new RunWinch(-0.5));
        rightBase_WinchOut.whenReleased(new RunWinch(0));

        JoystickButton rightBase_Left = new JoystickButton(joyRight, 16);
        rightBase_Left.whenPressed(new ExtendRamps(ExtendRamps.Side.LEFT));

        JoystickButton rightBase_Right = new JoystickButton(joyRight, 10);
        rightBase_Right.whenPressed(new ExtendRamps(ExtendRamps.Side.RIGHT));

        // secondary controls
        JoystickButton xbox_a = new JoystickButton(xbox, 1);
        xbox_a.whenPressed(new Roll(0.5));
        xbox_a.whenReleased(new StopRoll());

        JoystickButton xbox_b = new JoystickButton(xbox, 2);
        xbox_b.whenPressed(new Roll(-0.5));
        xbox_b.whenReleased(new StopRoll());

        JoystickButton xbox_x = new JoystickButton(xbox, 3);
        xbox_x.whenPressed(new Roll(-0.75));
        xbox_x.whenReleased(new StopRoll());

        JoystickButton xbox_y = new JoystickButton(xbox, 4);
        xbox_y.whenPressed(new InvertClawState());

        JoystickButton xbox_lb = new JoystickButton(xbox, 5);
        xbox_lb.whenReleased(new StopRoll());

        JoystickButton xbox_rb = new JoystickButton(xbox, 6);
        xbox_rb.whenReleased(new StopRoll());
    }

    public Joystick getJoyLeft() {
        return joyLeft;
    }

    public Joystick getJoyRight() {
        return joyRight;
    }

    public XboxController getXbox() {
        return xbox;
    }
}
