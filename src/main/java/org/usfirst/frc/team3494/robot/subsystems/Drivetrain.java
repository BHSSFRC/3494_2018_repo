package org.usfirst.frc.team3494.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
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

    private HRLVUltrasonicSensor uSonic;

    private Encoder encoderRight;
    private Encoder encoderLeft;
    private static final double DISTANCE_PER_PULSE = 1 / 256;

    private boolean teleop;
    public double pidTune;

    public Drivetrain() {
        super("Drivetrain", 0.025, 0, 0);
        double talon_P = 1.6;

        this.driveLeftMaster = new TalonSRX(RobotMap.DRIVE_LEFT_MASTER);
        this.driveLeftMaster.setNeutralMode(NeutralMode.Brake);
        this.driveLeftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
        // this.driveLeftMaster.setSensorPhase(true);
        this.driveLeftMaster.config_kP(0, talon_P, 10);
        this.driveLeftMaster.config_kF(0, 1 / ((RobotMap.PATH_MAX_SPEED * RobotMap.COUNTS_PER_METER / 10) * 4), 10);

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

        this.encoderLeft = new Encoder(RobotMap.ENCODER_LEFT_A, RobotMap.ENCODER_LEFT_B);
        this.encoderLeft.setDistancePerPulse(DISTANCE_PER_PULSE);
        this.encoderRight = new Encoder(RobotMap.ENCODER_RIGHT_A, RobotMap.ENCODER_RIGHT_B);
        this.encoderRight.setDistancePerPulse(DISTANCE_PER_PULSE);

        teleop = false;
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
        this.driveLeftMaster.set(ControlMode.PercentOutput, applyDeadband(left, 0.05));
        this.driveRightMaster.set(ControlMode.PercentOutput, applyDeadband(right, 0.05));
    }

    /**
     * Drives the drive train tank drive style, in velocity mode (encoder edges / decisecond.) The drive train will
     * continue to run until stopped with a method like {@link Drivetrain#StopDrive()}.
     *
     * @param left  The speed to drive the left side to.
     * @param right The speed to drive the right side to.
     */
    public void VelocityTank(double left, double right) {
        System.out.println("Target: " + left + ", " + right);
        System.out.println("Actual: " + this.driveLeftMaster.getSensorCollection().getQuadratureVelocity() + ", " + this.driveRightMaster.getSensorCollection().getQuadratureVelocity());
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
        xSpeed = limit(xSpeed);
        xSpeed = applyDeadband(xSpeed, 0.02);

        zRotation = limit(zRotation);
        zRotation = applyDeadband(zRotation, 0.02);

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

        driveLeftMaster.set(ControlMode.PercentOutput, limit(leftMotorOutput));
        driveRightMaster.set(ControlMode.PercentOutput, limit(rightMotorOutput));
    }

    public int getCountsLeft_Talon() {
        return this.driveLeftMaster.getSensorCollection().getQuadraturePosition();
    }

    public int getCountsRight_Talon() {
        return this.driveRightMaster.getSensorCollection().getQuadraturePosition();
    }

    public double getDistanceLeft_Talon() {
        return this.driveLeftMaster.getSensorCollection().getQuadraturePosition() * (1 / 4) * (DISTANCE_PER_PULSE);
    }

    public double getDistanceRight_Talon() {
        return this.driveRightMaster.getSensorCollection().getQuadraturePosition() * (1 / 4) * (DISTANCE_PER_PULSE);
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

    public void resetEncoders() {
        encoderRight.reset();
        encoderLeft.reset();
        this.driveRightMaster.getSensorCollection().setQuadraturePosition(0, 0);
        this.driveLeftMaster.getSensorCollection().setQuadraturePosition(0, 0);
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

    /**
     * Limit motor values to the -1.0 to +1.0 range.
     *
     * @param num The number to limit to [-1, 1].
     * @return The limited value.
     */
    private static double limit(double num) {
        if (num > 1.0) {
            return 1.0;
        }
        if (num < -1.0) {
            return -1.0;
        }
        return num;
    }

    /**
     * Returns 0.0 if the given value is within the specified range around zero. The remaining range
     * between the deadband and 1.0 is scaled from 0.0 to 1.0.
     *
     * @param value    value to clip
     * @param deadband range around zero
     * @return Zero if the value is in the deadband, or the value unchanged.
     * @author Worcester Polytechnic Institute
     */
    private double applyDeadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
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

    public static double nativeToRPS(double nat) {
        return nat * 10 / (256 * 4);
    }

    public static double rpsToNative(double rps) {
        return rps / 10 * (256 * 4);
    }

    public void PosDrive(double left, double right) {
        this.driveRightMaster.set(ControlMode.Position, right);
        this.driveLeftMaster.set(ControlMode.Position, left);
    }
}
