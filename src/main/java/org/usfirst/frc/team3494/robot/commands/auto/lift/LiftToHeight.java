package org.usfirst.frc.team3494.robot.commands.auto.lift;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team3494.robot.Robot;

public class LiftToHeight extends Command {
    private double height;

    public LiftToHeight(double h) {
        requires(Robot.lift);
        height = h;
    }

    @Override
    protected void execute() {
        if (height > Robot.lift.getHeight()) {
            Robot.lift.lift(.75);
        } else {
            Robot.lift.lift(-.25);
        }
    }

    @Override
    protected void end() {
        Robot.lift.lift(0);
    }

    @Override
    protected boolean isFinished() {
        return Robot.lift.getHeight() == this.height || (this.height > Robot.lift.getHeight() && Robot.lift.getHallTop()) || (this.height < Robot.lift.getHeight() && Robot.lift.getHallTop());
    }
}
