package org.usfirst.frc.team3494.robot.commands.auto.drive;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team3494.robot.Robot;

public class AngleTurn extends Command {

    private double angle;

    public AngleTurn(double angle) {
        this.angle = angle;
    }

    @Override
    protected void initialize() {
        Robot.driveTrain.setSetpoint(angle);
    }

    @Override
    protected void execute() {
        Robot.driveTrain.ArcadeDrive(0, Robot.driveTrain.getPidTune(), true);
    }

    @Override
    protected void end() {
        Robot.driveTrain.TankDrive(0, 0);
    }

    @Override
    protected boolean isFinished() {
        return Robot.driveTrain.onTarget();
    }
}
