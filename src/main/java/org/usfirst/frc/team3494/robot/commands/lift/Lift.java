package org.usfirst.frc.team3494.robot.commands.lift;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team3494.robot.Robot;

public class Lift extends Command {

    private static final double TOP_POSITION = Double.MAX_VALUE;
    private static final double EXCHANGE_POSITION = org.usfirst.frc.team3494.robot.subsystems.Lift.revsToCounts(org.usfirst.frc.team3494.robot.subsystems.Lift.inchesToRevs(2.0D));
    private static final double BOTTOM_POSITION = 0;

    public Lift() {
        requires(Robot.lift);
    }

    @Override
    protected void execute() {
        int pov = Robot.oi.getXbox().getPOV();
        double stick = -Robot.oi.getXbox().getY(GenericHID.Hand.kLeft);
        if (stick < 0) {
            Robot.lift.lift(stick / 2);
        } else {
            Robot.lift.lift(stick / 1.6);
        }
        /*
        if (pov == 0) {
            Robot.lift.unsafeLift(0.1);
        } else if (pov == 180) {
            Robot.lift.posLift(BOTTOM_POSITION);
        } else if (pov == 90 || pov == 270) {
            Robot.lift.posLift(EXCHANGE_POSITION);
        } else if (!Robot.lift.isPositionLifting()) {
            Robot.lift.lift(0);
        } */
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
