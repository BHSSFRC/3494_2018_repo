package org.usfirst.frc.team3494.robot.commands.auto.drive;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team3494.robot.Robot;
import org.usfirst.frc.team3494.robot.RobotMap;
import org.usfirst.frc.team3494.robot.util.FunctionalDoubleManager;
import org.usfirst.frc.team3494.robot.util.PIDIn;

/**
 * Drive the robot a given distance by encoder measurements.
 */
public class DistanceDrive extends Command {

    private double distance;
    private double speed;
    private PIDController pidController;
    private PIDIn pidIn;
    private FunctionalDoubleManager gyroOutput;

    /**
     * Constructor.
     *
     * @param distance The distance to drive, in feet.
     */
    public DistanceDrive(double distance) {
        this(distance, false);
    }

    /**
     * Constructor.
     *
     * @param distance The distance to drive, in feet.
     * @param fast     If driving at higher speed, set to true.
     */
    public DistanceDrive(double distance, boolean fast) {
        requires(Robot.driveTrain);
        this.distance = distance * 12 * RobotMap.EDGES_PER_INCH * .84; //the .84 is the multiple needed to accurately convert Edges to Inches
        this.speed = fast ? 0.5 : 0.25;

        gyroOutput = () -> Robot.ahrs.getFusedHeading();
        pidIn = new PIDIn(gyroOutput, PIDSourceType.kDisplacement);
        PIDOutput pidOut = (double output) -> Robot.driveTrain.TankDrive(speed + output, speed - output);
        //double Kp, double Ki, double Kd, PIDSource source, PIDOutput output

        pidController = new PIDController(0.01, 0.0, .1, pidIn, pidOut);

        pidController.setAbsoluteTolerance(.5);//3 is in degrees
        pidController.setInputRange(0, 360);
        pidController.setContinuous(true);

        pidController.setOutputRange(-.15, .15);
    }

    @Override
    protected void initialize() {
        Robot.driveTrain.resetEncoders();
        pidController.enable();
    }

    @Override
    protected void execute() {
        // Technically we just no-op here and let the PIDController drive.
        // We could also have a different lambda in the PIDController that set an instance field,
        // and drive the robot in here instead.
    }

    @Override
    protected void end() {
        Robot.driveTrain.StopDrive();
        pidController.disable();
    }

    @Override
    protected boolean isFinished() {
        return (Math.abs(Robot.driveTrain.getAverageCounts_Talon()) >= Math.abs(this.distance));
    }
}
