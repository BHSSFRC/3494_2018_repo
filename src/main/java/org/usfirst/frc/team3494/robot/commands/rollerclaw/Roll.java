package org.usfirst.frc.team3494.robot.commands.rollerclaw;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team3494.robot.Robot;

public class Roll extends Command {

    private boolean in;

    public Roll(boolean dir) {
        requires(Robot.rollerClaw);
        in = dir;
    }

    @Override
    protected void execute() {
        if (in) {
            Robot.rollerClaw.Rollerin();
        } else {
            Robot.rollerClaw.Rollerout();
        }
    }

    @Override
    protected boolean isFinished() {
        return true;
    }
}
