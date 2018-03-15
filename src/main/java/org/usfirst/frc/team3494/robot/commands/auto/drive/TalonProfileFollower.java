package org.usfirst.frc.team3494.robot.commands.auto.drive;

import com.ctre.phoenix.motion.SetValueMotionProfile;
import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Trajectory;
import org.usfirst.frc.team3494.robot.Robot;

/**
 * Command to follow a motion profile with Talon SRX MP.
 */
public class TalonProfileFollower extends Command {

    private static final int min_points = 128;
    private Trajectory trajectory;

    public TalonProfileFollower(Trajectory trajectory) {
        requires(Robot.driveTrain);
        this.trajectory = trajectory;
    }

    @Override
    protected void initialize() {
        Robot.driveTrain.startFillingLeft(Robot.pathfinderFormatToTalon(trajectory), trajectory.length());
        Robot.driveTrain.startFillingRight(Robot.pathfinderFormatToTalon(trajectory), trajectory.length());
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
