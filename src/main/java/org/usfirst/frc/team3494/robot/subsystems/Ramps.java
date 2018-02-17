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

    public Ramps() {
        super("Ramps");
        this.leftRamp = new DoubleSolenoid(RobotMap.LEFT_RAMP_FORWARD, RobotMap.LEFT_RAMP_REVERSE);
        this.rightRamp = new DoubleSolenoid(RobotMap.RIGHT_RAMP_FORWARD, RobotMap.RIGHT_RAMP_REVERSE);
    }

    @Override
    protected void initDefaultCommand() {
    }

    /**
     * Raise the ramps off the ground.
     */
    public void extend() {
        this.leftRamp.set(DoubleSolenoid.Value.kForward);
        this.rightRamp.set(DoubleSolenoid.Value.kForward);
    }

    /**
     * Put the ramps back on the ground.
     */
    public void retract() {
        this.leftRamp.set(DoubleSolenoid.Value.kReverse);
        this.rightRamp.set(DoubleSolenoid.Value.kReverse);
    }
}
