package org.usfirst.frc.team3494.robot.commands.rollerclaw;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team3494.robot.Robot;

public class SetClawState extends Command {
    private boolean value;

    public SetClawState(boolean v) {
        this.value = v;
        requires(Robot.rollerClaw);
    }

    @Override
    protected void execute() {
        Robot.rollerClaw.setRollerPist(this.value);
    }

    @Override
    protected boolean isFinished() {
        return true;
    }
}
