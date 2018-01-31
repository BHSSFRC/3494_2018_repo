package org.usfirst.frc.team3494.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team3494.robot.RobotMap;

public class Lights extends Subsystem {
    public double voltage;

    public Spark lightVoltage = new Spark(RobotMap.LIGHT_VOLTAGE);

    public Lights() {
        super("Lights");
    }

    @Override
    protected void initDefaultCommand() {
    }

    public void setLights(double voltage) {
        lightVoltage.set(voltage);
    }
}

