package org.usfirst.frc.team3494.robot.commands.auto.vision;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team3494.robot.Robot;
import org.usfirst.frc.team3494.robot.sensors.Limelight;

/**
 * Auton program to chase reflective items (or whatever is in pipeline zero.)
 */
public class ReflectivePursuit extends Command {
    private double lastTX;

    /**
     * Constructor.
     *
     * @param Tx A "guess" value for an initial horizontal displacement.
     */
    public ReflectivePursuit(double Tx) {
        requires(Robot.driveTrain);
        this.lastTX = Tx;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Robot.driveTrain.enable();
        Robot.limelight.setLEDs(Limelight.LIMELIGHT_LED_ON);
        Robot.limelight.setPipeline(0);
    }

    @Override
    protected void execute() {
        if (Robot.limelight.hasValidTarget()) {
            Robot.driveTrain.ArcadeDrive(0.65, Robot.driveTrain.getPidTune(), true);
            double tx = Robot.limelight.getXDistance();
            Robot.driveTrain.setSetpoint(Robot.ahrs.getYaw() + tx);
            SmartDashboard.putNumber("Drive PID setpoint", Robot.driveTrain.getSetpoint());
            lastTX = tx;
        } else {
            try {
                if (lastTX > 0) {
                    Robot.driveTrain.TankDrive(.25, -.25);
                } else if (lastTX < 0) {
                    Robot.driveTrain.TankDrive(-.25, .25);
                } else {
                    Robot.driveTrain.TankDrive(0.25, 0.25);
                }
            } catch (NullPointerException e) {
                Robot.driveTrain.StopDrive();
                System.out.println("oh no");
                e.printStackTrace();
            }
        }
    }

    @Override
    protected void end() {
        Robot.driveTrain.disable();
        Robot.driveTrain.StopDrive();
    }

    @Override
    protected boolean isFinished() {
        return (Robot.driveTrain.getSonicDistance() <= 24.0D);
    }
}
