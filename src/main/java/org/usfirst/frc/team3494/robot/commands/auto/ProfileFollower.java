package org.usfirst.frc.team3494.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import org.usfirst.frc.team3494.robot.Robot;

import java.io.File;

public class ProfileFollower extends Command {

    private Trajectory left;
    private Trajectory right;

    private double startTime;
    private int index;

    public ProfileFollower(String leftFile, String rightFile) {
        requires(Robot.driveTrain);
        left = Pathfinder.readFromCSV(new File(leftFile));
        right = Pathfinder.readFromCSV(new File(rightFile));
    }

    @Override
    protected void initialize() {
        Robot.driveTrain.resetEncoders();
        startTime = Timer.getFPGATimestamp() * 1000.0;
    }

    @Override
    protected void execute() {
        index = ((int) Math.floor(((Timer.getFPGATimestamp() * 1000.0) - startTime) / 50));

        double leftVelo = Robot.feetToMeters(left.segments[index].velocity);
        double rightVelo = Robot.feetToMeters(right.segments[index].velocity);
        Robot.driveTrain.VelocityTank(
                (Robot.metersToEdges(leftVelo)) / 10,
                (Robot.metersToEdges(rightVelo)) / 10);
    }

    @Override
    protected boolean isFinished() {
        return index + 1 >= left.segments.length;
    }
}
