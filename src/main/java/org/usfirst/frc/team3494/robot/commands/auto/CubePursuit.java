package org.usfirst.frc.team3494.robot.commands.auto;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team3494.robot.Robot;
import org.usfirst.frc.team3494.robot.util.Limelight;

import static org.usfirst.frc.team3494.robot.Robot.limelight;

/**
 * Auton program to chase Power Cubes (or whatever is in pipeline two.)
 */
public class CubePursuit extends Command {

    public CubePursuit() {
        requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Robot.driveTrain.enable();
        limelight.setLEDs(Limelight.LIMELIGHT_LED_OFF);
        limelight.setPipeline(1);
    }

    @Override
    protected void execute() {
        Robot.driveTrain.ArcadeDrive(0.65, Robot.driveTrain.pidTune, true);
        double tx = limelight.getXDistance();
        Robot.driveTrain.setSetpoint(Robot.ahrs.getYaw() + tx);
        SmartDashboard.putNumber("Drive PID setpoint", Robot.driveTrain.getSetpoint());
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
