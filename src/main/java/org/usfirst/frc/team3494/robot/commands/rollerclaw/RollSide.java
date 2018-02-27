package org.usfirst.frc.team3494.robot.commands.rollerclaw;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team3494.robot.Robot;
import org.usfirst.frc.team3494.robot.subsystems.Rollerclaw;

public class RollSide extends Command {

    private Rollerclaw.Rollers roller;
    private double power;

    public RollSide(Rollerclaw.Rollers rollers, double power) {
        requires(Robot.rollerClaw);
        this.roller = rollers;
        this.power = power;
    }

    @Override
    protected void execute() {
        Robot.rollerClaw.singleRoll(this.roller, this.power);
    }

    @Override
    protected boolean isFinished() {
        return true;
    }
}
