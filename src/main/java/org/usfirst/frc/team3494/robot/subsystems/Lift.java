package org.usfirst.frc.team3494.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team3494.robot.RobotMap;
import org.usfirst.frc.team3494.robot.sensors.HallEffectSensor;

/**
 * The lift subsystem. Contains methods for controlling the robot's lift.
 */
public class Lift extends PIDSubsystem {
    /**
     * The single motor that runs the lift up and down.
     */
    private TalonSRX liftMotor;

    private HallEffectSensor hallTop;
    private HallEffectSensor hallBottom;

    private static final double UNITS_PER_ROTATION = 4096.0D;

    private double pidTune;

    public Lift() {
        super("Lift", 0.001, 0, 0);
        this.setInputRange(-20365, 20365);
        this.setOutputRange(-0.4, 0.4);
        this.setAbsoluteTolerance(100.0D);
        this.getPIDController().setContinuous(false);
        this.pidTune = 0;

        liftMotor = new TalonSRX(RobotMap.LIFT_MOTOR);
        liftMotor.setNeutralMode(NeutralMode.Brake);
        liftMotor.setInverted(true);
        liftMotor.setSensorPhase(true);
        liftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        liftMotor.setSelectedSensorPosition(0, 0, 10);

        hallTop = new HallEffectSensor(RobotMap.LIFT_HALL_TOP);
        hallBottom = new HallEffectSensor(RobotMap.LIFT_HALL_BOT);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new org.usfirst.frc.team3494.robot.commands.lift.Lift());
    }

    @Override
    public void periodic() {
        if (this.hallBottom.isActive()) {
            this.liftMotor.setSelectedSensorPosition(0, 0, 0);
        }
    }

    /**
     * Runs the lift motor at the specified power.
     *
     * @param power The power to run the lift at. Should be a {@code double}
     *              between -1 and 1.
     */
    public void lift(double power) {
        if (power > 0 && !this.getHallTop()) {
            liftMotor.set(ControlMode.PercentOutput, power);
        } else if (power < 0 && !this.getHallBottom()) {
            liftMotor.set(ControlMode.PercentOutput, power);
        } else {
            liftMotor.set(ControlMode.PercentOutput, 0);
        }
    }

    public int getHeight_Edges() {
        return liftMotor.getSensorCollection().getQuadraturePosition();
    }

    public double getHeight_Revolutions() {
        return ((double) this.getHeight_Edges()) / UNITS_PER_ROTATION;
    }

    public double getHeight_Inches() {
        return this.getHeight_Revolutions() * 1.4D * Math.PI;
    }

    public boolean getHallTop() {
        return this.hallTop.isActive();
    }

    public boolean getHallBottom() {
        if (this.hallBottom.isActive()) {
            this.liftMotor.getSensorCollection().setPulseWidthPosition(0, 0);
        }
        return this.hallBottom.isActive();
    }

    public static double inchesToRevs(double inches) {
        return inches / (1.4 * Math.PI);
    }

    public static double revsToCounts(double revs) {
        return revs * 4096;
    }

    @Override
    protected double returnPIDInput() {
        return this.getHeight_Edges();
    }

    @Override
    protected void usePIDOutput(double output) {
        this.pidTune = output;
    }

    public double getPidTune() {
        return this.pidTune;
    }
}
