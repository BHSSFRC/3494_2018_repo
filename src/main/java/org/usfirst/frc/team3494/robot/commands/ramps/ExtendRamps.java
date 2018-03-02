package org.usfirst.frc.team3494.robot.commands.ramps;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team3494.robot.Robot;

public class ExtendRamps extends Command {
    private Side side;

    public ExtendRamps(Side s) {
        requires(Robot.ramps);
        this.side = s;
    }

    @Override
    protected void execute() {
        if (Robot.ramps.getClaw().equals(DoubleSolenoid.Value.kForward)) {
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
        } // else no-op
    }

    protected boolean isFinished() {
        return true;
    }

    public enum Side {
        LEFT, RIGHT, BOTH
    }
}
