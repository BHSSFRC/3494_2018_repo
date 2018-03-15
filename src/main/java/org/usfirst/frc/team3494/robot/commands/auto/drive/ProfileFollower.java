package org.usfirst.frc.team3494.robot.commands.auto.drive;

import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import org.usfirst.frc.team3494.robot.Robot;

import java.io.File;

public class ProfileFollower extends Command {

    private Trajectory left;
    private Trajectory right;

    private int index;

    public ProfileFollower(String leftFile, String rightFile) {
        requires(Robot.driveTrain);
        left = Pathfinder.readFromCSV(new File(leftFile));
        right = Pathfinder.readFromCSV(new File(rightFile));
    }

    @Override
    protected void initialize() {
        Robot.driveTrain.resetEncoders();
        Robot.getTimer().reset();
        Robot.getTimer().start();
    }

    @Override
    protected void execute() {
        index = ((int) Math.floor(((Robot.getTimer().get() * 1000.0)) / 200));

        double leftVelo = Robot.feetToMeters(left.segments[index].velocity);
        double rightVelo = Robot.feetToMeters(right.segments[index].velocity);
        Robot.driveTrain.VelocityTank(
                (Robot.metersToEdges(leftVelo)) / 10,
                (Robot.metersToEdges(rightVelo)) / 10);
    }

    @Override
    protected void end() {
        Robot.driveTrain.TankDrive(0, 0);
    }

    @Override
    protected boolean isFinished() {
        return index + 1 >= left.segments.length;
    }
}
