package org.usfirst.frc.team3494.robot.commands.auto.drive;

import com.ctre.phoenix.motion.SetValueMotionProfile;
import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import org.usfirst.frc.team3494.robot.Robot;

import java.io.File;

/**
 * Command to follow a motion profile with Talon SRX MP. Similar to {@link ProfileFollower}, but loads
 * the points onto the Talons and uses Talon-side motion profiling.
 */
public class TalonProfileFollower extends Command {

    private static final int min_points = 40;
    private double[][] trajectory_left;
    private double[][] trajectory_right;

    /**
     * Constructor.
     *
     * @param leftFile  The path to the left side trajectory CSV.
     * @param rightFile The path to the right side trajectory CSV.
     */
    public TalonProfileFollower(String leftFile, String rightFile) {
        this(Pathfinder.readFromCSV(new File(leftFile)), Pathfinder.readFromCSV(new File(rightFile)));
    }

    /**
     * Constructor.
     *
     * @param traj_l The left side Pathfinder trajectory.
     * @param traj_r The right side Pathfinder trajectory.
     */
    public TalonProfileFollower(Trajectory traj_l, Trajectory traj_r) {
        this(Robot.pathfinderFormatToTalon(traj_l), Robot.pathfinderFormatToTalon(traj_r));
    }

    /**
     * Constructor. Takes "talon" format profiles
     * (i.e. lists of lists of doubles in the form [position, velocity, dt].)
     *
     * @param traj_l The left side trajectory.
     * @param traj_r The right side trajectory.
     */
    public TalonProfileFollower(double[][] traj_l, double[][] traj_r) {
        requires(Robot.driveTrain);
        this.trajectory_left = traj_l;
        this.trajectory_right = traj_r;
    }

    @Override
    protected void initialize() {
        Robot.driveTrain.StopDrive();
        Robot.driveTrain.resetEncoders();
        System.out.println("Filling talons...");
        Robot.driveTrain.startFillingLeft(trajectory_left, trajectory_left.length);
        Robot.driveTrain.startFillingRight(trajectory_right, trajectory_right.length);
        while (Robot.driveTrain.getLeftMpStatus().btmBufferCnt < min_points || Robot.driveTrain.getRightMpStatus().btmBufferCnt < min_points) {
            Robot.driveTrain.periodic();
        }
        System.out.println("Talons filled (enough)!");
    }

    @Override
    protected void execute() {
        Robot.driveTrain.leftMpControl(SetValueMotionProfile.Enable);
        Robot.driveTrain.rightMpControl(SetValueMotionProfile.Enable);
    }

    @Override
    protected void end() {
        System.out.println("Done MP driving!");
        Robot.driveTrain.leftMpControl(SetValueMotionProfile.Disable);
        Robot.driveTrain.rightMpControl(SetValueMotionProfile.Disable);
        Robot.driveTrain.StopDrive();
    }

    @Override
    protected boolean isFinished() {
        return (Robot.driveTrain.getLeftMpStatus().btmBufferCnt == 0 || Robot.driveTrain.getRightMpStatus().btmBufferCnt == 0);
    }
}
