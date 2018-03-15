package org.usfirst.frc.team3494.robot.commands.auto.drive;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        index = ((int) Math.floor((Robot.getTimer().get() * 1000.0) / 50.0));
        SmartDashboard.putNumber("ProfileFollower index", index);

        double leftVelo = left.segments[index].velocity;
        double rightVelo = right.segments[index].velocity;
        Robot.driveTrain.VelocityTank(
                (Robot.feetToEdges(leftVelo)) / 10,
                (Robot.feetToEdges(rightVelo)) / 10);
    }

    @Override
    protected void end() {
        Robot.driveTrain.StopDrive();
    }

    @Override
    protected boolean isFinished() {
        return index + 1 >= left.segments.length;
    }
}
