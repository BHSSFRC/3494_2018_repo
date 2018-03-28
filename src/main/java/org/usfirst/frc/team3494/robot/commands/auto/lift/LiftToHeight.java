package org.usfirst.frc.team3494.robot.commands.auto.lift;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team3494.robot.Robot;

public class LiftToHeight extends Command {
    private double height;

    public LiftToHeight(double h) {
        requires(Robot.lift);
        height = h;
    }

    @Override
    protected void initialize() {
        Robot.lift.setSetpoint(height);
        Robot.lift.enable();
    }

    @Override
    protected void execute() {
        SmartDashboard.putNumber("Lift target", Robot.lift.getSetpoint());
        SmartDashboard.putNumber("Lift error", Robot.lift.getPIDController().getError());
        SmartDashboard.putNumber("Lift PID output", Robot.lift.getPidTune());
        SmartDashboard.putBoolean("Lift on target", Robot.lift.onTarget());
        Robot.lift.lift(Robot.lift.getPidTune());
    }

    @Override
    protected void end() {
        Robot.lift.disable();
        Robot.lift.lift(0);
    }

    @Override
    protected boolean isFinished() {
        return Robot.lift.onTarget() || Robot.lift.getHallTop() || Robot.lift.getHallBottom();
    }
}
