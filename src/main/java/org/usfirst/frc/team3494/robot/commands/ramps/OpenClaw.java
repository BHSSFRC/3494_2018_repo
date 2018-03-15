package org.usfirst.frc.team3494.robot.commands.ramps;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team3494.robot.Robot;

public class OpenClaw extends Command {

    public OpenClaw() {
        requires(Robot.ramps);
    }

    @Override
    protected void execute() {
        Robot.ramps.openClaw();
    }

    @Override
    protected boolean isFinished() {
        return true;
    }
}
