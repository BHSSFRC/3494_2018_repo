package org.usfirst.frc.team3494.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team3494.robot.RobotMap;

/**
 * The roller claw subsystem. Contains methods for controlling the robot's roller claw.
 */
public class Rollerclaw extends Subsystem {

    private VictorSPX rollerLeft;
    private VictorSPX rollerRight;
    private Solenoid rollerPist;

    public Rollerclaw() {
        super("Rollerclaw");
        rollerLeft = new VictorSPX(RobotMap.ROLLER_LEFT);
        rollerRight = new VictorSPX(RobotMap.ROLLER_RIGHT);
        this.rollerPist = new Solenoid(RobotMap.ROLLER_PISTON_FORWARD);
    }

    @Override
    protected void initDefaultCommand() {
    }

    /**
     * Runs the rollers inwards.
     */
    public void rollIn() {
        //rollerLeft.set(ControlMode.PercentOutput, .75); // this is dumb
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
     * Run the roller claw at {@code power}.
     *
     * @param power The power to run the roller claw at.
     */
    public void customRoll(double power) {
        rollerLeft.set(ControlMode.PercentOutput, power);
        rollerRight.set(ControlMode.PercentOutput, power);
    }

    /**
     * Sets the piston on the claw (opening for {@code false} and closing for {@code true}.)
     *
     * @param v The state to set the claw to.
     */
    public void setRollerPist(boolean v) {
        this.rollerPist.set(v);
    }

    /**
     * Get the state of the piston on the claw.
     *
     * @return {@code false} for open and {@code true} for closed.
     */
    public boolean getRollerPist() {
        return this.rollerPist.get();
    }
}
