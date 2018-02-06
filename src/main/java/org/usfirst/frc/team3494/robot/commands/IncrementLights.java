package org.usfirst.frc.team3494.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team3494.robot.Robot;

public class IncrementLights extends Command {
    public IncrementLights() {
        requires(Robot.lights);
    }

    @Override
    protected void execute() {
        if (Robot.lights.getVoltage() == .99) {
            Robot.lights.setLights(-.99);
        } else {
            Robot.lights.setLights(Robot.lights.getVoltage() + .02);
        }
    }

    @Override
    protected boolean isFinished() {
        return true;
    }
}
