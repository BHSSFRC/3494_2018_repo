package org.usfirst.frc.team3494.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team3494.robot.RobotMap;

/**
 * The... LED strips? Contains methods for controlling the robot's LEDs (apparently.)
 */
public class Lights extends Subsystem {
    private double voltage;

    /**
     * The LED controller.
     */
    private Spark lightVoltage = new Spark(RobotMap.LIGHT_VOLTAGE);

    public Lights() {
        super("Lights");
        voltage = -.99;
        lightVoltage.set(-.99);
    }

    @Override
    protected void initDefaultCommand() {
    }

    /**
     * Sets the LEDs to a specified power code. Range from -0.99 to 0.99, odds only.
     *
     * @param voltage The code to set the LEDs to.
     */
    public void setLights(double voltage) {
        this.voltage = voltage;
        lightVoltage.set(voltage);
    }

    /**
     * Returns the current LED status.
     *
     * @return The current status of the LEDs as a double from -0.99 to 0.99.
     */
    public double getVoltage() {
        return voltage;
    }
}

