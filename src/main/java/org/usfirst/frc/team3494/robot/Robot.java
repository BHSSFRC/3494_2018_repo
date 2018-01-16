package org.usfirst.frc.team3494.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team3494.robot.subsystems.Drivetrain;
import org.usfirst.frc.team3494.robot.util.Limelight;


public class Robot extends IterativeRobot {
    private String fieldData;
    /**
     * Instance of {@link OI}. No subsystem should require this. However, you
     * can read button values from it.
     */
    public static OI oi;

    public static AHRS ahrs;
    public static Limelight limelight;
    UsbCamera camera_0;

    public static Drivetrain driveTrain;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        oi = new OI();
        ahrs = new AHRS(SPI.Port.kMXP);
        limelight = new Limelight();

        driveTrain = new Drivetrain();

        SmartDashboard.putNumber("Limelight pipeline", 0);
        camera_0 = CameraServer.getInstance().startAutomaticCapture("flaming bagpipes", 0);
    }

    @Override
    public void autonomousInit() {
        fieldData = DriverStation.getInstance().getGameSpecificMessage();
        limelight.setLEDs(Limelight.LIMELIGHT_LED_ON);
        int pipe = (int) SmartDashboard.getNumber("Limelight pipeline", 0);
        if (pipe == 1) {
            limelight.setLEDs(Limelight.LIMELIGHT_LED_OFF);
        }
        limelight.setPipeline(pipe);
        camera_0.setExposureManual(20);
        Robot.driveTrain.enable();
    }

    @Override
    public void autonomousPeriodic() {
        Robot.driveTrain.ArcadeDrive(0.65, Robot.driveTrain.pidTune, true);
        double tx = limelight.getXDistance();
        Robot.driveTrain.setSetpoint(Robot.ahrs.getYaw() + tx);
        SmartDashboard.putNumber("Drive PID setpoint", Robot.driveTrain.getSetpoint());
    }

    @Override
    public void teleopInit() {
        limelight.setLEDs(Limelight.LIMELIGHT_LED_OFF);
    }

    @Override
    public void disabledInit() {
        limelight.setLEDs(Limelight.LIMELIGHT_LED_OFF);
    }
}
