package org.usfirst.frc.team3494.robot.commands.auto.drive;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team3494.robot.Robot;

public class DistanceDrive extends Command {

    private double distance;

    public DistanceDrive(double distance) {
        requires(Robot.driveTrain);
        this.distance = Robot.feetToEdges(distance);
    }

    @Override
    protected void initialize() {
        Robot.driveTrain.resetEncoders();
    }

    @Override
    protected void execute() {
        if (distance > 0) {
            Robot.driveTrain.TankDrive(0.5, 0.5);
        } else {
            Robot.driveTrain.TankDrive(-0.5, -0.5);
        }
    }

    @Override
    protected boolean isFinished() {
        return Math.abs(this.distance) >= Math.abs(Robot.driveTrain.getAverageCounts_Talon());
    }
}
