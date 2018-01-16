package org.usfirst.frc.team3494.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team3494.robot.RobotMap;
import org.usfirst.frc.team3494.robot.commands.drive.Drive;

public class Drivetrain extends Subsystem {
    private TalonSRX driveLeftMaster;
    private TalonSRX driveLeftFollowOne;
    private TalonSRX driveLeftFollowTwo;

    private TalonSRX driveRightMaster;
    private TalonSRX driveRightFollowOne;
    private TalonSRX driveRightFollowTwo;

    public Drivetrain() {
        super("Drivetrain");

        this.driveLeftMaster = new TalonSRX(RobotMap.DRIVE_LEFT_MASTER);
        this.driveLeftFollowOne = new TalonSRX(RobotMap.DRIVE_LEFT_FOLLOW_ONE);
        this.driveLeftFollowTwo = new TalonSRX(RobotMap.DRIVE_LEFT_FOLLOW_TWO);

        this.driveRightMaster = new TalonSRX(RobotMap.DRIVE_RIGHT_MASTER);
        this.driveRightFollowOne = new TalonSRX(RobotMap.DRIVE_RIGHT_FOLLOW_ONE);
        this.driveRightFollowTwo = new TalonSRX(RobotMap.DRIVE_RIGHT_FOLLOW_TWO);
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
            driveLeftMaster.set(ControlMode.PercentOutput, left);
            driveRightMaster.set(ControlMode.PercentOutput, right);
        }
    }

    /**
     * Stops all drive motors. Does not require re-enabling motors after use.
     *
     * @since 0.0.0
     */
    public void StopDrive() {
        driveLeftMaster.set(ControlMode.Current, 0);
        driveRightMaster.set(ControlMode.Current, 0);
    }
}
