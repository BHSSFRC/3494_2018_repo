package org.usfirst.frc.team3494.robot.commands.auto.tests;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.modifiers.TankModifier;
import org.usfirst.frc.team3494.robot.Robot;

public class PathTestOne extends Command {

    private Trajectory leftTraj;
    private Trajectory rightTraj;

    private double startTime;
    private int index;

    public PathTestOne() {
        requires(Robot.driveTrain);
    }

    @Override
    protected void initialize() {
        Robot.driveTrain.resetEncoders();

        TankModifier modifier = Robot.pathBuilder.getCenterToLeftMod();

        leftTraj = modifier.getLeftTrajectory();
        rightTraj = modifier.getRightTrajectory();

        startTime = Timer.getFPGATimestamp() * 1000.0;
    }

    @Override
    protected void execute() {
        index = ((int) Math.floor(((Timer.getFPGATimestamp() * 1000.0) - startTime) / 50));

        double leftVelo = leftTraj.segments[index].velocity;
        double rightVelo = rightTraj.segments[index].velocity;
        Robot.driveTrain.VelocityTank(
                (Robot.metersToEdges(leftVelo)) / 10,
                (Robot.metersToEdges(rightVelo)) / 10);
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
