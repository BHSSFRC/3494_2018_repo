package org.usfirst.frc.team3494.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team3494.robot.RobotMap;

public class Lights extends Subsystem {
    private double voltage;

    private Spark lightVoltage = new Spark(RobotMap.LIGHT_VOLTAGE);

    public Lights() {
        super("Lights");
        voltage = -.99;
        lightVoltage.set(-.99);
    }

    @Override
    protected void initDefaultCommand() {
    }

    public void setLights(double voltage) {
        this.voltage = voltage;
        lightVoltage.set(voltage);
    }

    public double getVoltage() {
        return voltage;
    }
}

