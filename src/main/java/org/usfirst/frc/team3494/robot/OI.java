package org.usfirst.frc.team3494.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import org.usfirst.frc.team3494.robot.commands.IncrementLights;
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

    Joystick joyLeft = new Joystick(RobotMap.JOYSTICK_LEFT);
    Joystick joyRight = new Joystick(RobotMap.JOYSTICK_RIGHT);

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
    public OI() {
        JoystickButton leftTrigger = new JoystickButton(joyLeft, 1);
        leftTrigger.whenPressed(new Roll(true));
        leftTrigger.whenReleased(new StopRoll());

        JoystickButton rightTrigger = new JoystickButton(joyRight, 1);
        rightTrigger.whenPressed(new Roll(false));
        rightTrigger.whenReleased(new StopRoll());

        JoystickButton leftThumb = new JoystickButton(joyLeft, 2);
        leftThumb.whenPressed(new IncrementLights());
    }

    public Joystick getJoyLeft() {
        return joyLeft;
    }

    public Joystick getJoyRight() {
        return joyRight;
    }
}
