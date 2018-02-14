package org.usfirst.frc.team3494.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team3494.robot.RobotMap;

public class Lift extends Subsystem {
    private TalonSRX liftMotor;

    public Lift() {
        liftMotor = new TalonSRX(RobotMap.LIFT_MOTOR);
    }

    @Override
    protected void initDefaultCommand() {
    }

    public void lift(double power) {
        liftMotor.set(ControlMode.PercentOutput, power);
    }
}

