package org.usfirst.frc.team3494.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import org.usfirst.frc.team3494.robot.Robot;
import org.usfirst.frc.team3494.robot.RobotMap;
import org.usfirst.frc.team3494.robot.commands.drive.Drive;

public class Drivetrain extends PIDSubsystem {
    private TalonSRX driveLeftMaster;
    private TalonSRX driveLeftFollowOne;
    private TalonSRX driveLeftFollowTwo;
    private TalonSRX[] leftSide;

    private TalonSRX driveRightMaster;
    private TalonSRX driveRightFollowOne;
    private TalonSRX driveRightFollowTwo;
    private TalonSRX[] rightSide;
    private boolean teleop;
    private double pidTune;

    public Drivetrain() {
        super("Drivetrain", 0.4, 0, 0.5);

        this.driveLeftMaster = new TalonSRX(RobotMap.DRIVE_LEFT_MASTER);
        this.driveLeftFollowOne = new TalonSRX(RobotMap.DRIVE_LEFT_FOLLOW_ONE);
        this.driveLeftFollowOne.set(ControlMode.Follower, this.driveLeftMaster.getBaseID());
        this.driveLeftFollowTwo = new TalonSRX(RobotMap.DRIVE_LEFT_FOLLOW_TWO);
        this.driveLeftFollowTwo.set(ControlMode.Follower, this.driveLeftMaster.getBaseID());

        leftSide = new TalonSRX[]{
                this.driveLeftMaster, this.driveLeftFollowOne, this.driveLeftFollowTwo
        };

        this.driveRightMaster = new TalonSRX(RobotMap.DRIVE_RIGHT_MASTER);
        this.driveRightFollowOne = new TalonSRX(RobotMap.DRIVE_RIGHT_FOLLOW_ONE);
        this.driveRightFollowOne.set(ControlMode.Follower, this.driveRightMaster.getBaseID());
        this.driveRightFollowTwo = new TalonSRX(RobotMap.DRIVE_RIGHT_FOLLOW_TWO);
        this.driveRightFollowTwo.set(ControlMode.Follower, this.driveRightMaster.getBaseID());

        rightSide = new TalonSRX[]{
                this.driveRightMaster, this.driveRightFollowOne, this.driveRightFollowTwo
        };

        teleop = false;
        pidTune = 0;
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new Drive());
    }

    /**
     * Drives the driveTrain tank drive style. The driveTrain will continue to
     * run until stopped with a method like {@link Drivetrain#StopDrive()}.
     *
     * @param left  The power to drive the left side. Should be a {@code double}
     *              between 0 and 1.
     * @param right The power to drive the right side. Should be a {@code double}
     *              between 0 and 1.
     */
    public void TankDrive(double left, double right) {
        if (Math.abs(left) > RobotMap.DRIVE_TOLERANCE && Math.abs(right) > RobotMap.DRIVE_TOLERANCE) {
            this.driveLeftMaster.set(ControlMode.PercentOutput, left);
            this.driveRightMaster.set(ControlMode.PercentOutput, right);
        }
    }

    /**
     * Stops all drive motors. Does not require re-enabling motors after use.
     *
     * @since 0.0.0
     */
    public void StopDrive() {
        this.driveLeftMaster.set(ControlMode.PercentOutput, 0);
        this.driveRightMaster.set(ControlMode.PercentOutput, 0);
    }

    @Override
    protected double returnPIDInput() {
        if (!teleop) {
            return Robot.ahrs.getYaw();
        } else {
            return Robot.ahrs.getAngle();
        }
    }

    @Override
    protected void usePIDOutput(double output) {
        this.pidTune = output;
    }

    public double getPidTune() {
        return pidTune;
    }
}
