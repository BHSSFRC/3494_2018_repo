package org.usfirst.frc.team3494.robot.commands.auto.drive;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team3494.robot.Robot;

public class AngleDrive extends Command {

    private double angle;

    public AngleDrive(double angle) {
        requires(Robot.driveTrain);
        this.angle = angle;
    }

    @Override
    protected void initialize() {
        Robot.ahrs.reset();
        Robot.driveTrain.setSetpoint(this.angle);
        Robot.driveTrain.enable();
    }

    @Override
    protected void execute() {
        Robot.driveTrain.ArcadeDrive(0, Robot.driveTrain.getPidTune(), true);
    }

    @Override
    protected void end() {
        Robot.driveTrain.disable();
        Robot.driveTrain.StopDrive();
    }

    @Override
    protected boolean isFinished() {
        return Robot.driveTrain.onTarget();
    }
}
