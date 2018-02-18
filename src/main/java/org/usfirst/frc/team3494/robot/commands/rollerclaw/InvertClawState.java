package org.usfirst.frc.team3494.robot.commands.rollerclaw;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team3494.robot.Robot;

public class InvertClawState extends Command {
    private boolean value;

    public InvertClawState() {
        requires(Robot.rollerClaw);
    }

    @Override
    protected void execute() {
        Robot.rollerClaw.setRollerPist(!Robot.rollerClaw.getRollerPist());
    }

    @Override
    protected boolean isFinished() {
        return true;
    }
}
