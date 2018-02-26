package org.usfirst.frc.team3494.robot.commands.ramps;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team3494.robot.Robot;

public class ExtendRamps extends Command {
    public ExtendRamps() {
        requires(Robot.ramps);
    }

    @Override
    protected void execute() {
        Robot.ramps.extend();
    }

    protected boolean isFinished() {
        return true;
    }
}
