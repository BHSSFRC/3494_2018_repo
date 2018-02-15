package org.usfirst.frc.team3494.robot.commands.lift;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team3494.robot.Robot;

public class RunLift extends Command {
    double speed;

    public RunLift(double speed) {
        this.speed = speed;
        requires(Robot.lift);
    }

    @Override
    protected void execute() {
        Robot.lift.lift(speed);
    }

    @Override
    protected boolean isFinished() {
        return true;
    }
}
