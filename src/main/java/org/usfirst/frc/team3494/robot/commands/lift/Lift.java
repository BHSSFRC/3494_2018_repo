package org.usfirst.frc.team3494.robot.commands.lift;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team3494.robot.Robot;

public class Lift extends Command {
    public Lift() {
        requires(Robot.lift);
    }

    @Override
    protected void execute() {
        double stick = Robot.applyDeadband(-Robot.oi.getXbox().getY(GenericHID.Hand.kLeft), 0.01);
        if (stick < 0) {
            Robot.lift.lift(stick / 2.0);
        } else {
            Robot.lift.lift(stick / 1.3);
        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
