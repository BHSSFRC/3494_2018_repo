package org.usfirst.frc.team3494.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team3494.robot.RobotMap;

/**
 * The ramps subsystem. Contains methods for controlling the robot's ramps.
 */
public class Ramps extends Subsystem {
    /**
     * Solenoid running the left ramp.
     */
    private DoubleSolenoid leftRamp;
    /**
     * Solenoid running the right ramp.
     */
    private DoubleSolenoid rightRamp;

    private DoubleSolenoid rampClaw;

    public Ramps() {
        super("Ramps");
        this.leftRamp = new DoubleSolenoid(RobotMap.LEFT_RAMP_FORWARD, RobotMap.LEFT_RAMP_REVERSE);
        this.leftRamp.set(DoubleSolenoid.Value.kReverse);
        this.rightRamp = new DoubleSolenoid(RobotMap.RIGHT_RAMP_FORWARD, RobotMap.RIGHT_RAMP_REVERSE);
        this.rightRamp.set(DoubleSolenoid.Value.kReverse);
        this.rampClaw = new DoubleSolenoid(RobotMap.RAMP_CLAW_FORWARD, RobotMap.RAMP_CLAW_REVERSE);
        this.rampClaw.set(DoubleSolenoid.Value.kReverse);
    }

    @Override
    protected void initDefaultCommand() {
    }

    /**
     * Raise the ramps off the ground.
     */
    public void extend() {
        this.setLeftRamp(DoubleSolenoid.Value.kForward);
        this.setRightRamp(DoubleSolenoid.Value.kForward);
    }

    /**
     * Put the ramps back on the ground.
     */
    public void retract() {
        this.setLeftRamp(DoubleSolenoid.Value.kReverse);
        this.setRightRamp(DoubleSolenoid.Value.kReverse);
    }

    public void openClaw() {
        this.rampClaw.set(DoubleSolenoid.Value.kForward);
    }

    public void setLeftRamp(DoubleSolenoid.Value v) {
        this.leftRamp.set(v);
    }

    public void setRightRamp(DoubleSolenoid.Value v) {
        this.rightRamp.set(v);
    }
}
