package org.usfirst.frc.team3494.robot.commands.auto.drive;

import com.ctre.phoenix.motion.SetValueMotionProfile;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team3494.robot.Robot;

public class TalonProfileFollower extends Command {

    private static final int min_points = 128;

    public TalonProfileFollower() {
        requires(Robot.driveTrain);
    }

    @Override
    protected void initialize() {
        // TODO: get actual profiles to fill into Talons
        // Robot.driveTrain.startFillingLeft();
        // Robot.driveTrain.startFillingRight();
        while (Robot.driveTrain.getLeftMpStatus().btmBufferCnt < min_points || Robot.driveTrain.getRightMpStatus().btmBufferCnt < min_points) {
            continue; // loop until done filling
        }
    }

    @Override
    protected void execute() {
        Robot.driveTrain.leftMpControl(SetValueMotionProfile.Enable);
        Robot.driveTrain.rightMpControl(SetValueMotionProfile.Enable);
    }

    @Override
    protected void end() {
        Robot.driveTrain.leftMpControl(SetValueMotionProfile.Disable);
        Robot.driveTrain.rightMpControl(SetValueMotionProfile.Disable);
    }

    @Override
    protected boolean isFinished() {
        boolean finished = false;
        if (Robot.driveTrain.getLeftMpStatus().activePointValid && Robot.driveTrain.getLeftMpStatus().isLast) {
            finished = true;
        } else if (Robot.driveTrain.getRightMpStatus().activePointValid && Robot.driveTrain.getRightMpStatus().isLast) {
            finished = true;
        }
        return finished;
    }
}
