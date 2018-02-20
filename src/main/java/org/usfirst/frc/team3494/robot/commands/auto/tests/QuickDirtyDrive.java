package org.usfirst.frc.team3494.robot.commands.auto.tests;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team3494.robot.Robot;

public class QuickDirtyDrive extends Command {

    public QuickDirtyDrive() {
        requires(Robot.driveTrain);
    }

    @Override
    protected void initialize() {
        Robot.driveTrain.resetEncoders();
    }

    @Override
    protected void execute() {
        Robot.driveTrain.TankDrive(-1, 1);
    }

    @Override
    protected boolean isFinished() {
        return Robot.driveTrain.getCountsRight_Talon() >= 100;
    }
}
