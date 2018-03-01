package org.usfirst.frc.team3494.robot.commands.lift;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team3494.robot.Robot;

public class Lift extends Command {

    public Lift() {
        requires(Robot.lift);
    }

    @Override
    protected void execute() {
        int pov = Robot.oi.getXbox().getPOV();
        if (pov == -1) {
            Robot.lift.lift(0);
        } else if (pov == 0) {
            Robot.lift.lift(0.85);
        } else if (pov == 180) {
            Robot.lift.lift(-0.25);
        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
