package org.usfirst.frc.team3494.robot.commands.ramps;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team3494.robot.Robot;
import org.usfirst.frc.team3494.robot.subsystems.Ramps;

public class RetractRamps extends Command {
    private Ramps.Side side;

    public RetractRamps(Ramps.Side s) {
        requires(Robot.ramps);
        this.side = s;
    }

    @Override
    protected void execute() {
        switch (this.side) {
            case LEFT:
                Robot.ramps.setLeftRamp(DoubleSolenoid.Value.kReverse);
                break;
            case RIGHT:
                Robot.ramps.setRightRamp(DoubleSolenoid.Value.kReverse);
                break;
            case BOTH:
                Robot.ramps.retract();
                break;
        }
    }

    @Override
    protected boolean isFinished() {
        return true;
    }
}
