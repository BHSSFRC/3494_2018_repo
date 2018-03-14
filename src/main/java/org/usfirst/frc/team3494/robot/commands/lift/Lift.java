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
        double stick = -Robot.oi.getXbox().getY(GenericHID.Hand.kLeft);
        if (stick < 0) {
            Robot.lift.lift(stick / 2);
        } else {
            Robot.lift.lift(stick / 1.6);
        }


        int pov = Robot.oi.getXbox().getPOV();
        if (pov == 0) {
            Robot.lift.unsafeLift(0.1);
        } else if (pov == 180) {
            Robot.lift.unsafeLift(-0.1);
        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
