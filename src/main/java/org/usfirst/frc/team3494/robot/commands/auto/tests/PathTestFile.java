package org.usfirst.frc.team3494.robot.commands.auto.tests;

import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import org.usfirst.frc.team3494.robot.Robot;

import java.io.File;

public class PathTestFile extends Command {
    private Trajectory leftTraj;
    private Trajectory rightTraj;

    private double startTime;
    private int index;

    public PathTestFile() {
        requires(Robot.driveTrain);
        leftTraj = Pathfinder.readFromCSV(new File("/home/lvuser/paths/center/center2right_left.csv"));
        rightTraj = Pathfinder.readFromCSV(new File("/home/lvuser/paths/center/center2right_right.csv"));
    }

    @Override
    protected void initialize() {
        Robot.driveTrain.resetEncoders();
        Robot.getTimer().reset();
        startTime = Robot.getTimer().get() * 1000.0;
    }

    @Override
    protected void execute() {
        index = ((int) Math.floor(((Robot.getTimer().get() * 1000.0) - startTime) / 50));

        double leftVelo = leftTraj.segments[index].velocity;
        double rightVelo = rightTraj.segments[index].velocity;
        Robot.driveTrain.VelocityTank(
                (Robot.feetToEdges(leftVelo)) / 10,
                (Robot.feetToEdges(rightVelo)) / 10);
    }

    @Override
    protected boolean isFinished() {
        return index + 1 >= leftTraj.segments.length;
    }

    @Override
    protected void end() {
        Robot.driveTrain.StopDrive();
    }
}
