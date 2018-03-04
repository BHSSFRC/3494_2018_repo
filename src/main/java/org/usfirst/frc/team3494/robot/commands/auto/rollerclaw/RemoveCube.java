package org.usfirst.frc.team3494.robot.commands.auto.rollerclaw;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team3494.robot.Robot;

public class RemoveCube extends Command {

    public RemoveCube() {
        requires(Robot.rollerClaw);
    }

    @Override
    protected void initialize() {
        Robot.getTimer().reset();
    }

    @Override
    protected void execute() {
        Robot.rollerClaw.rollOut();
    }

    @Override
    protected void end() {
        Robot.rollerClaw.rollStop();
    }

    @Override
    protected boolean isFinished() {
        return Robot.getTimer().get() >= 5;
    }
}
