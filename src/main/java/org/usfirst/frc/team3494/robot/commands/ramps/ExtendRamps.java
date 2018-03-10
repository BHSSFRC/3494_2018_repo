package org.usfirst.frc.team3494.robot.commands.ramps;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team3494.robot.Robot;
import org.usfirst.frc.team3494.robot.subsystems.Ramps;

public class ExtendRamps extends Command {
    private Ramps.Side side;

    public ExtendRamps(Ramps.Side s) {
        requires(Robot.ramps);
        this.side = s;
    }

    @Override
    protected void execute() {
        switch (this.side) {
            case LEFT:
                Robot.ramps.setLeftRamp(DoubleSolenoid.Value.kForward);
                break;
            case RIGHT:
                Robot.ramps.setRightRamp(DoubleSolenoid.Value.kForward);
                break;
            case BOTH:
                Robot.ramps.extend();
                break;
        }
    }

    protected boolean isFinished() {
        return true;
    }
}
