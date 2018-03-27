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
        double stick = Robot.applyDeadband(-Robot.oi.getXbox().getY(GenericHID.Hand.kLeft), 0.01);
        if (stick != 0) {
            if (stick < 0) {
                Robot.lift.lift(stick / 2);
            } else {
                Robot.lift.lift(stick / 1.6);
            }
        }
        if (pov == 180) {
            if (Robot.lift.getHeight_Edges() != 0) {
                if (Robot.lift.getHeight_Edges() > 0 && !Robot.lift.getHallBottom()) {
                    Robot.lift.lift(-0.4);
                } else {
                    Robot.lift.lift(0.4);
                }
            } else {
                Robot.lift.lift(0);
            }
        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
