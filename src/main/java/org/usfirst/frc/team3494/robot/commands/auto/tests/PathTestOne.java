package org.usfirst.frc.team3494.robot.commands.auto.tests;

import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;
import org.usfirst.frc.team3494.robot.Robot;

public class PathTestOne extends Command {

    private EncoderFollower left;
    private EncoderFollower right;

    public PathTestOne() {
        requires(Robot.driveTrain);
    }

    @Override
    protected void initialize() {
        TankModifier modifier = Robot.pathBuilder.getCenterToLeftMod();

        left = new EncoderFollower(modifier.getLeftTrajectory());
        left.configureEncoder(Robot.driveTrain.getCountsLeft(), 256, 4.875);
        left.configurePIDVA(1.0, 0.0, 0.0, 1 / 1.7, 0);

        right = new EncoderFollower(modifier.getRightTrajectory());
        right.configureEncoder(Robot.driveTrain.getCountsRight(), 256, 4.875);
        right.configurePIDVA(1.0, 0.0, 0.0, 1 / 1.7, 0);
    }

    @Override
    protected void execute() {
        double leftVal = left.calculate(Robot.driveTrain.getCountsLeft());
        double rightVal = right.calculate(Robot.driveTrain.getCountsRight());

        double gyro_heading = Robot.ahrs.getAngle();    // Assuming the gyro is giving a value in degrees
        double desired_heading = Pathfinder.r2d(left.getHeading());  // Should also be in degrees

        double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
        double turn = 0.8 * (-1.0 / 80.0) * angleDifference;

        Robot.driveTrain.TankDrive(leftVal + turn, rightVal - turn);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
