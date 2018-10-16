package org.usfirst.frc.team3494.robot.commands.auto.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team3494.robot.Robot;
import org.usfirst.frc.team3494.robot.RobotMap;

/**
 * Drive the robot a given distance by encoder measurements.
 */
public class DistanceDrive extends Command {

    private double distance;
    private boolean fast;
    private Timer timer;

    /**
     * Constructor.
     *
     */
    /**
     * Constructor.
     *
     * @param distance The distance to drive, in feet.
     * @param fast     If driving at higher speed, set to true.
     */
    public DistanceDrive(double distance, boolean fast) {
        requires(Robot.driveTrain);
        this.distance = distance * 12 * RobotMap.EDGES_PER_INCH * RobotMap.FUDGE_FACTOR;
        this.fast = fast;
        this.timer = new Timer();

    }

    @Override
    protected void initialize() {
        Robot.driveTrain.resetEncoders();
        timer.start();
    }

    @Override
    protected void execute() {
        double speed;
        if (fast) {
            speed = 0.5;
        } else {
            speed = 0.25;
        }

        Robot.driveTrain.TankDrive(speed, speed);
    }

    @Override
    protected void end() {
        Robot.driveTrain.StopDrive();
        timer.stop();
    }

    @Override
    protected boolean isFinished() {
        return (Math.abs(Robot.driveTrain.getAverageCounts_Talon()) >= Math.abs(this.distance) || timer.get() < 5 ) ;

    }
}
