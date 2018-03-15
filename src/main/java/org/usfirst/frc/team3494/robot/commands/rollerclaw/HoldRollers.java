package org.usfirst.frc.team3494.robot.commands.rollerclaw;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team3494.robot.Robot;

public class HoldRollers extends Command {

    public HoldRollers() {
        requires(Robot.rollerClaw);
    }

    @Override
    protected void execute() {
        Robot.rollerClaw.customRoll(0.5);
    }

    @Override
    protected boolean isFinished() {
        return true;
    }
}
