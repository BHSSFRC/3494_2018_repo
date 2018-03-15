package org.usfirst.frc.team3494.robot.commands.rollerclaw;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team3494.robot.Robot;

public class Roll extends Command {

    private double power;

    /**
     * Constructor.
     *
     * @param power The power to roll both sides of the claw at.
     */
    public Roll(double power) {
        requires(Robot.rollerClaw);
        this.power = power;
    }

    @Override
    protected void execute() {
        Robot.rollerClaw.customRoll(power);
    }

    @Override
    protected boolean isFinished() {
        return true;
    }
}
