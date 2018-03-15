package org.usfirst.frc.team3494.robot.commands.auto.drive;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team3494.robot.Robot;

/**
 * Drive the robot a given distance by encoder measurements.
 */
public class DistanceDrive extends Command {

    private double distance;
    private boolean fast;

    /**
     * Constructor.
     *
     * @param distance The distance to drive, in feet.
     */
    public DistanceDrive(double distance) {
        this(distance, false);
    }

    public DistanceDrive(double distance, boolean fast) {
        requires(Robot.driveTrain);
        this.distance = Robot.feetToEdges(distance);
        this.fast = fast;
    }

    @Override
    protected void initialize() {
        Robot.driveTrain.resetEncoders();
    }

    @Override
    protected void execute() {
        double speed;
        if (fast) {
            speed = 0.5;
        } else {
            speed = 0.25;
        }
        if (distance > 0) {
            Robot.driveTrain.TankDrive(speed, speed);
        } else {
            Robot.driveTrain.TankDrive(-speed, -speed);
        }
    }

    @Override
    protected void end() {
        Robot.driveTrain.StopDrive();
    }

    @Override
    protected boolean isFinished() {
        return Math.abs(this.distance) <= Math.abs(Robot.driveTrain.getAverageCounts_Talon());
    }
}
