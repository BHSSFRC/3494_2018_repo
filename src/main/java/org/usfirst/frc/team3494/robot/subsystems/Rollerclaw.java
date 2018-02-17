package org.usfirst.frc.team3494.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team3494.robot.RobotMap;

/**
 * The roller claw subsystem. Contains methods for controlling the robot's roller claw.
 */
public class Rollerclaw extends Subsystem {

    private VictorSPX rollerLeft;
    private VictorSPX rollerRight;
    private DoubleSolenoid rollerPist;

    public Rollerclaw() {
        super("Rollerclaw");
        rollerLeft = new VictorSPX(RobotMap.ROLLER_LEFT);
        rollerRight = new VictorSPX(RobotMap.ROLLER_RIGHT);
        this.rollerPist = new DoubleSolenoid(RobotMap.ROLLER_PISTON_FORWARD, RobotMap.ROLLER_PISTON_REVERSE);
    }

    @Override
    protected void initDefaultCommand() {
    }

    /**
     * Runs the rollers inwards.
     */
    public void rollIn() {
        rollerLeft.set(ControlMode.PercentOutput, .75);
        rollerRight.set(ControlMode.PercentOutput, .75);
    }

    /**
     * Stops the rollers.
     */
    public void rollStop() {
        rollerLeft.set(ControlMode.PercentOutput, 0);
        rollerRight.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Runs the rollers outwards.
     */
    public void rollOut() {
        rollerLeft.set(ControlMode.PercentOutput, -.75);
        rollerRight.set(ControlMode.PercentOutput, -.75);
    }

    /**
     * Sets the piston on the claw (opening for for {@link edu.wpi.first.wpilibj.DoubleSolenoid.Value#kReverse kReverse}
     * and closing for {@link edu.wpi.first.wpilibj.DoubleSolenoid.Value#kForward kForward}.
     *
     * @param v The state to set the claw to.
     */
    public void setRollerPist(DoubleSolenoid.Value v) {
        this.rollerPist.set(v);
    }
}
