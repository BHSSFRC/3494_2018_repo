package org.usfirst.frc.team3494.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team3494.robot.RobotMap;
import org.usfirst.frc.team3494.robot.sensors.HallEffectSensor;

/**
 * The lift subsystem. Contains methods for controlling the robot's lift.
 */
public class Lift extends Subsystem {
    /**
     * The single motor that runs the lift up and down.
     */
    private TalonSRX liftMotor;

    private HallEffectSensor hallTop;
    private HallEffectSensor hallBottom;

    private static final double UNITS_PER_ROTATION = 4096;

    public Lift() {
        liftMotor = new TalonSRX(RobotMap.LIFT_MOTOR);
        liftMotor.setNeutralMode(NeutralMode.Coast);
        liftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

        hallTop = new HallEffectSensor(RobotMap.LIFT_HALL_TOP);
        hallBottom = new HallEffectSensor(RobotMap.LIFT_HALL_BOT);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new org.usfirst.frc.team3494.robot.commands.lift.Lift());
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

    public void unsafeLift(double power) {
        liftMotor.set(ControlMode.PercentOutput, power);
    }

    public double getHeight_Edges() {
        return liftMotor.getSelectedSensorPosition(0);
    }

    public double getHeight() {
        return this.getHeight_Edges() / UNITS_PER_ROTATION;
    }

    public boolean getHallTop() {
        return this.hallTop.isActive();
    }

    public boolean getHallBottom() {
        return this.hallBottom.isActive();
    }
}
