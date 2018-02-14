package org.usfirst.frc.team3494.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team3494.robot.RobotMap;

public class Ramps extends Subsystem {
    private DoubleSolenoid leftRamp;
    private DoubleSolenoid rightRamp;

    public Ramps() {
        super("Ramps");
        this.leftRamp = new DoubleSolenoid(RobotMap.LEFT_RAMP_FORWARD, RobotMap.LEFT_RAMP_REVERSE);
        this.rightRamp = new DoubleSolenoid(RobotMap.RIGHT_RAMP_FORWARD, RobotMap.RIGHT_RAMP_REVERSE);
    }

    @Override
    protected void initDefaultCommand() {
    }

    public void extend() {
        this.leftRamp.set(DoubleSolenoid.Value.kForward);
        this.rightRamp.set(DoubleSolenoid.Value.kForward);
    }

    public void retract() {
        this.leftRamp.set(DoubleSolenoid.Value.kReverse);
        this.rightRamp.set(DoubleSolenoid.Value.kReverse);
    }
}
