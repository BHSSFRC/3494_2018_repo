package org.usfirst.frc.team3494.robot.commands.rollerclaw;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team3494.robot.Robot;

public class StopRoll extends Command {
    public StopRoll() {
        requires(Robot.rollerClaw);
    }

    @Override
    protected void execute() {
        Robot.rollerClaw.Rollerstop();
    }

    @Override
    protected boolean isFinished() {
        return true;
    }
}
