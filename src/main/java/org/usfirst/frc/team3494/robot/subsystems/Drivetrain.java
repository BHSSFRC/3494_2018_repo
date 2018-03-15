package org.usfirst.frc.team3494.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import org.usfirst.frc.team3494.robot.Robot;
import org.usfirst.frc.team3494.robot.RobotMap;
import org.usfirst.frc.team3494.robot.commands.drive.Drive;
import org.usfirst.frc.team3494.robot.sensors.HRLVUltrasonicSensor;

/**
 * The drive train subsystem. Contains methods for controlling the robot's drive train.
 * Also includes PID angle control via the
 * {@link com.kauailabs.navx.frc.AHRS AHRS} mounted to the RoboRIO and {@link PIDSubsystem}.
 */
public class Drivetrain extends PIDSubsystem {
    /**
     * Master Talon SRX, left side.
     */
    private TalonSRX driveLeftMaster;
    /**
     * Follower Talon SRX, left side.
     */
    private TalonSRX driveLeftFollowOne;
    /**
     * Additional follower Talon SRX, left side.
     */
    private TalonSRX driveLeftFollowTwo;

    /**
     * Master Talon SRX, right side.
     */
    private TalonSRX driveRightMaster;
    /**
     * Follower Talon SRX, right side.
     */
    private TalonSRX driveRightFollowOne;
    /**
     * Additional follower Talon SRX, right side.
     */
    private TalonSRX driveRightFollowTwo;
    /**
     * The ultrasonic sensor used for ending some vision commands.
     */
    private HRLVUltrasonicSensor uSonic;

    /**
     * The turn value to use with PID angle driving via {@link Drivetrain#ArcadeDrive(double, double, boolean)}.
     */
    private double pidTune;

    public Drivetrain() {
        super("Drivetrain", 0.025, 0, 0);
        double talon_P = 2.0D;

        this.driveLeftMaster = new TalonSRX(RobotMap.DRIVE_LEFT_MASTER);
        this.driveLeftMaster.setNeutralMode(NeutralMode.Brake);
        this.driveLeftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
        this.driveLeftMaster.config_kP(0, talon_P, 10);
        this.driveLeftMaster.config_kF(0, 1 / ((RobotMap.PATH_MAX_SPEED * RobotMap.COUNTS_PER_METER / 10.0) * 4.0), 10);

        this.driveLeftFollowOne = new TalonSRX(RobotMap.DRIVE_LEFT_FOLLOW_ONE);
        this.driveLeftFollowOne.set(ControlMode.Follower, RobotMap.DRIVE_LEFT_MASTER);
        this.driveLeftFollowOne.setNeutralMode(NeutralMode.Brake);

        this.driveLeftFollowTwo = new TalonSRX(RobotMap.DRIVE_LEFT_FOLLOW_TWO);
        this.driveLeftFollowTwo.set(ControlMode.Follower, RobotMap.DRIVE_LEFT_MASTER);
        this.driveLeftFollowTwo.setNeutralMode(NeutralMode.Brake);

        this.driveRightMaster = new TalonSRX(RobotMap.DRIVE_RIGHT_MASTER);
        this.driveRightMaster.setNeutralMode(NeutralMode.Brake);
        this.driveRightMaster.setInverted(true);
        this.driveRightMaster.setSensorPhase(true);
        this.driveRightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
        this.driveRightMaster.config_kP(0, talon_P, 10);
        this.driveRightMaster.config_kF(0, 1 / ((RobotMap.PATH_MAX_SPEED * RobotMap.COUNTS_PER_METER / 10) * 4), 10);

        this.driveRightFollowOne = new TalonSRX(RobotMap.DRIVE_RIGHT_FOLLOW_ONE);
        this.driveRightFollowOne.set(ControlMode.Follower, RobotMap.DRIVE_RIGHT_MASTER);
        this.driveRightFollowOne.setNeutralMode(NeutralMode.Brake);
        this.driveRightFollowOne.setInverted(true);

        this.driveRightFollowTwo = new TalonSRX(RobotMap.DRIVE_RIGHT_FOLLOW_TWO);
        this.driveRightFollowTwo.set(ControlMode.Follower, RobotMap.DRIVE_RIGHT_MASTER);
        this.driveRightFollowTwo.setNeutralMode(NeutralMode.Brake);
        this.driveRightFollowTwo.setInverted(true);

        this.uSonic = new HRLVUltrasonicSensor(RobotMap.USONIC_PIN);

        // config pid loop
        pidTune = 0;
        double outRange = 0.9;
        setInputRange(-180, 180);
        setOutputRange(-outRange, outRange);
        getPIDController().setContinuous(true);
        setPercentTolerance(1);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new Drive());
    }

    /**
     * Drives the drive train tank drive style. The driveTrain will continue to
     * run until stopped with a method like {@link Drivetrain#StopDrive()}.
     *
     * @param left  The power to drive the left side. Should be a {@code double}
     *              between -1 and 1.
     * @param right The power to drive the right side. Should be a {@code double}
     *              between -1 and 1.
     */
    public void TankDrive(double left, double right) {
        this.driveLeftMaster.set(ControlMode.PercentOutput, Robot.applyDeadband(left, 0.05));
        this.driveRightMaster.set(ControlMode.PercentOutput, Robot.applyDeadband(right, 0.05));
    }

    /**
     * Drives the drive train tank drive style, in velocity mode (encoder edges / decisecond.) The drive train will
     * continue to run until stopped with a method like {@link Drivetrain#StopDrive()}.
     *
     * @param left  The speed to drive the left side to.
     * @param right The speed to drive the right side to.
     */
    public void VelocityTank(double left, double right) {
        this.driveLeftMaster.set(ControlMode.Velocity, left);
        this.driveRightMaster.set(ControlMode.Velocity, right);
    }

    /**
     * Arcade drive method for differential drive platform.
     *
     * @param xSpeed        The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
     * @param zRotation     The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
     *                      positive.
     * @param squaredInputs If set, decreases the input sensitivity at low speeds.
     * @author Worcester Polytechnic Institute
     */
    public void ArcadeDrive(double xSpeed, double zRotation, boolean squaredInputs) {
        xSpeed = Robot.limit(xSpeed, 1);
        xSpeed = Robot.applyDeadband(xSpeed, 0.02);

        zRotation = Robot.limit(zRotation, 1);
        zRotation = Robot.applyDeadband(zRotation, 0.02);

        // Square the inputs (while preserving the sign) to increase fine control
        // while permitting full power.
        if (squaredInputs) {
            xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
            zRotation = Math.copySign(zRotation * zRotation, zRotation);
        }

        double leftMotorOutput;
        double rightMotorOutput;

        double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

        if (xSpeed >= 0.0) {
            // First quadrant, else second quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            } else {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            }
        } else {
            // Third quadrant, else fourth quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            } else {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            }
        }

        driveLeftMaster.set(ControlMode.PercentOutput, Robot.limit(leftMotorOutput, 1));
        driveRightMaster.set(ControlMode.PercentOutput, Robot.limit(rightMotorOutput, 1));
    }

    /**
     * Returns the distance from a wall as given by the ultrasonic sensor.
     *
     * @return The distance to a wall if the ultrasonic sensor is facing one.
     */
    public double getSonicDistance() {
        return this.uSonic.getDistance();
    }

    public double getSonicVoltage() {
        return this.uSonic.getVoltage();
    }

    /**
     * The number of encoder edges on the left side of the drivetrain.
     *
     * @return The number of encoder edges on the left side of the drivetrain.
     */
    public int getCountsLeft_Talon() {
        return this.driveLeftMaster.getSensorCollection().getQuadraturePosition();
    }

    /**
     * The number of encoder edges on the right side of the drivetrain.
     *
     * @return The number of encoder edges on the right side of the drivetrain.
     */
    public int getCountsRight_Talon() {
        return this.driveRightMaster.getSensorCollection().getQuadraturePosition();
    }

    /**
     * The average of the edges measured by each drivetrain side.
     *
     * @return The average distance traveled by the drivetrain.
     */
    public double getAverageCounts_Talon() {
        return (this.getCountsLeft_Talon() + this.getCountsRight_Talon()) / 2;
    }

    /**
     * The left drivetrain distance.
     *
     * @return The number of encoder revolutions on the drivetrain left side.
     */
    public double getDistanceLeft_Talon() {
        return ((double) this.getCountsLeft_Talon() / 4) / 256;
    }

    /**
     * The right drivetrain distance.
     *
     * @return The number of encoder revolutions on the drivetrain right side.
     */
    public double getDistanceRight_Talon() {
        return ((double) this.getCountsRight_Talon() / 4) / 256;
    }

    /**
     * The average drivetrain distance.
     *
     * @return The average number of encoder revolutions.
     */
    public double getAverageDistance_Talon() {
        return ((this.getDistanceLeft_Talon() + this.getDistanceRight_Talon()) / 2);
    }

    /**
     * @return The left side velocity, in edges per 100ms (decisecond.)
     */
    public double getVelocityLeft() {
        return this.driveLeftMaster.getSensorCollection().getQuadratureVelocity();
    }

    /**
     * @return The right side velocity, in edges per 100ms (decisecond.)
     */
    public double getVelocityRight() {
        return this.driveRightMaster.getSensorCollection().getQuadratureVelocity();
    }

    /**
     * Sets the number of encoder edges and revolutions on both sides to zero.
     */
    public void resetEncoders() {
        this.driveRightMaster.getSensorCollection().setQuadraturePosition(0, 0);
        this.driveLeftMaster.getSensorCollection().setQuadraturePosition(0, 0);
    }

    /**
     * Stops all drive motors. Does not require re-enabling motors after use.
     */
    public void StopDrive() {
        this.driveLeftMaster.set(ControlMode.PercentOutput, 0);
        this.driveRightMaster.set(ControlMode.PercentOutput, 0);
    }

    @Override
    protected double returnPIDInput() {
        return Robot.ahrs.getYaw();
    }

    @Override
    protected void usePIDOutput(double output) {
        this.pidTune = output;
    }

    /**
     * Returns the turn value from the drivetrain PID controller. Should be used for driving with {@link Drivetrain#ArcadeDrive(double, double, boolean)}.
     *
     * @return The turn value to use for reaching the PID setpoint.
     */
    public double getPidTune() {
        return pidTune;
    }

    /**
     * Convert native Talon units to revs/second.
     *
     * @param nat The value to convert in edges/decisecond.
     * @return The converted value in revolutions/second.
     */
    public static double nativeToRPS(double nat) {
        return nat * 10 / (256 * 4);
    }

    /**
     * Convert revs/second to native Talon units.
     *
     * @param rps The value to convert in revs/second.
     * @return The converted value in edges/decisecond.
     */
    public static double rpsToNative(double rps) {
        return rps / 10 * (256 * 4);
    }
}
