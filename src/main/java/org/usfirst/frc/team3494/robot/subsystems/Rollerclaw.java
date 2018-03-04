package org.usfirst.frc.team3494.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team3494.robot.RobotMap;
import org.usfirst.frc.team3494.robot.commands.rollerclaw.RollSide;

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
        rollerRight.setInverted(true);
        this.rollerPist = new Solenoid(RobotMap.ROLLER_PISTON);
    }

    @Override
    protected void initDefaultCommand() {
        this.setDefaultCommand(new RollSide(0.9));
    }

    /**
     * Runs the rollers inwards.
     */
    public void rollIn() {
        this.customRoll(0, 0.75);
    }

    /**
     * Stops the rollers.
     */
    public void rollStop() {
        this.customRoll(0);
    }

    /**
     * Runs the rollers outwards.
     */
    public void rollOut() {
        this.customRoll(-0.5);
    }

    /**
     * Run the roller claw at {@code power}.
     *
     * @param power The power to run the roller claw at.
     */
    public void customRoll(double power) {
        this.customRoll(power, power);
    }

    /**
     * Run both sides of the roller claw at given speeds.
     *
     * @param p_left  The power to run the left side at.
     * @param p_right The power to run the right side at.
     */
    public void customRoll(double p_left, double p_right) {
        rollerLeft.set(ControlMode.PercentOutput, p_left);
        rollerRight.set(ControlMode.PercentOutput, p_right);
    }

    /**
     * Run single or both sides of the roller claw. Unlike {@link Rollerclaw#customRoll(double, double)} and {@link Rollerclaw#customRoll(double)},
     * this method can be used to run a single side without changing the other.
     *
     * @param roller The {@link Rollers roller} to control.
     * @param power  The power to run the roller at.
     */
    public void singleRoll(Rollers roller, double power) {
        switch (roller) {
            case LEFT:
                this.rollerLeft.set(ControlMode.PercentOutput, power);
                break;
            case RIGHT:
                this.rollerRight.set(ControlMode.PercentOutput, power);
                break;
            case All:
                this.customRoll(power);
                break;
        }
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

    public enum Rollers {
        LEFT, RIGHT, All
    }
}
