package org.usfirst.frc.team3494.robot.commands.auto;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import org.usfirst.frc.team3494.robot.Robot;
import org.usfirst.frc.team3494.robot.commands.auto.lift.LiftToHeight;

public class AutoInitial extends CommandGroup {
    public AutoInitial() {
        addParallel(new LiftToHeight(300000));
        addParallel(new RollerWinch());
    }

    private static class RollerWinch extends Command {
        RollerWinch() {
            requires(Robot.rollerClaw);
        }

        @Override
        protected void initialize() {
            Robot.getTimer().reset();
        }

        @Override
        protected void execute() {
            Robot.rollerClaw.runWinch(1.0);
        }

        @Override
        protected void end() {
            Robot.rollerClaw.runWinch(0);
        }

        @Override
        protected boolean isFinished() {
            return Robot.getTimer().get() >= 1.0D;
        }
    }
}
