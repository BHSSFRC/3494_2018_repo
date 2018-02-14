package org.usfirst.frc.team3494.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team3494.robot.RobotMap;

public class Rollerclaw extends Subsystem {

    private TalonSRX rollerLeft;
    private TalonSRX rollerRight;
    private DoubleSolenoid rollerPist;

    public Rollerclaw() {
        super("Rollerclaw");
        rollerLeft = new TalonSRX(RobotMap.ROLLER_LEFT);
        rollerRight = new TalonSRX(RobotMap.ROLLER_RIGHT);
        this.rollerPist = new DoubleSolenoid(RobotMap.ROLLER_PISTON_FORWARD, RobotMap.ROLLER_PISTON_REVERSE);
    }

    @Override
    protected void initDefaultCommand() {
    }

    public void Rollerin() {
        rollerLeft.set(ControlMode.PercentOutput, .75);
        rollerRight.set(ControlMode.PercentOutput, .75);
    }

    public void Rollerstop() {
        rollerLeft.set(ControlMode.PercentOutput, 0);
        rollerRight.set(ControlMode.PercentOutput, 0);
    }

    public void Rollerout() {
        rollerLeft.set(ControlMode.PercentOutput, -.75);
        rollerRight.set(ControlMode.PercentOutput, -.75);
    }

    public void setRollerPist(DoubleSolenoid.Value v) {
        this.rollerPist.set(v);
    }
}
