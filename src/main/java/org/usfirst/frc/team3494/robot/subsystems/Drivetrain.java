package org.usfirst.frc.team3494.robot.subsystems;

import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team3494.robot.RobotMap;
import org.usfirst.frc.team3494.robot.commands.drive.Drive;

public class Drivetrain extends Subsystem {
    public TalonSRX driveLeftMaster = new TalonSRX(RobotMap.DRIVE_LEFT_MASTER);
    public TalonSRX driveRightMaster = new TalonSRX(RobotMap.DRIVE_RIGHT_MASTER);

    public Drivetrain() {
        super("Drivetrain");
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
            driveLeftMaster.set(left);
            driveRightMaster.set(right);
        }
    }

    /**
     * Stops all drive motors. Does not require re-enabling motors after use.
     *
     * @since 0.0.0
     */
    public void StopDrive() {
        driveLeftMaster.stopMotor();
        driveRightMaster.stopMotor();
    }
}
