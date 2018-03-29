package org.usfirst.frc.team3494.robot.commands.ramps;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team3494.robot.Robot;

public class ToggleClaw extends Command {

    public ToggleClaw() {
        requires(Robot.ramps);
    }

    @Override
    protected void execute() {
        if (Robot.ramps.getClaw().equals(DoubleSolenoid.Value.kReverse)) {
            Robot.ramps.openClaw();
        } else {
            Robot.ramps.closeClaw();
        }
    }

    @Override
    protected boolean isFinished() {
        return true;
    }
}
