package org.usfirst.frc.team3494.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team3494.robot.RobotMap;

public class Rollerclaw extends Subsystem {

    public TalonSRX rollerLeft = new TalonSRX(RobotMap.ROLLER_LEFT);
    public TalonSRX rollerRight = new TalonSRX(RobotMap.ROLLER_RIGHT);

    public Rollerclaw() {
        super("Rollerclaw");
    }

    @Override
    protected void initDefaultCommand() {

    }

    public void Rollerin() {
        rollerLeft.set(ControlMode.PercentOutput, .75);
        rollerRight.set(ControlMode.PercentOutput, .75);
    }

    public void Rollerout() {
        rollerLeft.set(ControlMode.PercentOutput, -.75);
        rollerRight.set(ControlMode.PercentOutput, -.75);
    }
}
