package org.usfirst.frc.team3494.robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team3494.robot.Robot;

/**
 * Command for operator control of the {@link Robot#driveTrain drivetrain}. Runs effectively infinitely.
 */
public class Drive extends Command {

    public Drive() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        Robot.driveTrain.TankDrive(
                -Robot.oi.getJoyLeft().getY(),
                Robot.oi.getJoyRight().getY()
        );
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
