package org.usfirst.frc.team3494.robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team3494.robot.Robot;

/**
 * Command for operator control of the {@link Robot#driveTrain drivetrain}. Runs effectively infinitely.
 */
public class Drive extends Command {

    private Fronts front = Fronts.CLAW;

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
        int pov = Robot.oi.getJoyLeft().getPOV();
        if (pov == 0) {
            this.front = Fronts.CLAW;
        } else if (pov == 180) {
            this.front = Fronts.BATTERY;
        }

        if (front.equals(Fronts.CLAW)) {
            Robot.driveTrain.TankDrive(
                    -Robot.oi.getJoyLeft().getY(),
                    -Robot.oi.getJoyRight().getY()
            );
        } else {
            Robot.driveTrain.TankDrive(
                    Robot.oi.getJoyRight().getY(),
                    Robot.oi.getJoyLeft().getY()
            );
        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    private enum Fronts {
        CLAW, BATTERY
    }
}
