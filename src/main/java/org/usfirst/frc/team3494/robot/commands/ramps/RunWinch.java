package org.usfirst.frc.team3494.robot.commands.ramps;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team3494.robot.Robot;

public class RunWinch extends Command {

    private double speed;

    public RunWinch(double speed) {
        requires(Robot.ramps);
        this.speed = speed;
    }

    @Override
    protected void execute() {
        Robot.ramps.setWinch(speed);
    }

    @Override
    protected boolean isFinished() {
        return true;
    }
}
