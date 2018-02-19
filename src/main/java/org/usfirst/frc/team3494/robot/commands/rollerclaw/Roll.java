package org.usfirst.frc.team3494.robot.commands.rollerclaw;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team3494.robot.Robot;

public class Roll extends Command {

    private boolean in;

    /**
     * Constructor.
     *
     * @param dir The direction to roll. Use {@code false} for out and {@code true} for in.
     */
    public Roll(boolean dir) {
        requires(Robot.rollerClaw);
        in = dir;
    }

    @Override
    protected void execute() {
        if (in) {
            Robot.rollerClaw.rollIn();
        } else {
            Robot.rollerClaw.rollOut();
        }
    }

    @Override
    protected boolean isFinished() {
        return true;
    }
}
