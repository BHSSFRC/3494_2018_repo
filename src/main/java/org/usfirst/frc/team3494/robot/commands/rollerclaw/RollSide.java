package org.usfirst.frc.team3494.robot.commands.rollerclaw;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team3494.robot.Robot;
import org.usfirst.frc.team3494.robot.subsystems.Rollerclaw;

public class RollSide extends Command {

    private double power;

    public RollSide(double power) {
        requires(Robot.rollerClaw);
        this.power = power;
    }

    @Override
    protected void execute() {
        if (Robot.oi.getXbox().getBumper(GenericHID.Hand.kLeft) && Robot.oi.getXbox().getBumper(GenericHID.Hand.kRight)) {
            Robot.rollerClaw.customRoll(power);
        } else if (Robot.oi.getXbox().getBumper(GenericHID.Hand.kLeft)) {
            Robot.rollerClaw.singleRoll(Rollerclaw.Rollers.LEFT, this.power);
        } else if (Robot.oi.getXbox().getBumper(GenericHID.Hand.kRight)) {
            Robot.rollerClaw.singleRoll(Rollerclaw.Rollers.RIGHT, this.power);
        } // if none of above are true, execute ends without doing anything
    }

    @Override
    protected boolean isFinished() {
        return false; // run unless interrupted (see also: Drive)
    }
}
