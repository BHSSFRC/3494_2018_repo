package org.usfirst.frc.team3494.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team3494.robot.RobotMap;

/**
 * The lift subsystem. Contains methods for controlling the robot's lift.
 */
public class Lift extends Subsystem {
    /**
     * The single motor that runs the lift up and down.
     */
    private TalonSRX liftMotor;

    public Lift() {
        liftMotor = new TalonSRX(RobotMap.LIFT_MOTOR);
        liftMotor.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    protected void initDefaultCommand() {
    }

    /**
     * Runs the lift motor at the specified power.
     *
     * @param power The power to run the lift at. Should be a {@code double}
     *              between -1 and 1.
     */
    public void lift(double power) {
        liftMotor.set(ControlMode.PercentOutput, power);
    }
}

