package org.usfirst.frc.team3494.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team3494.robot.commands.auto.CubePursuit;
import org.usfirst.frc.team3494.robot.commands.auto.ReflectivePursuit;
import org.usfirst.frc.team3494.robot.subsystems.Drivetrain;
import org.usfirst.frc.team3494.robot.subsystems.Rollerclaw;
import org.usfirst.frc.team3494.robot.util.Limelight;


public class Robot extends IterativeRobot {
    private String fieldData;
    /**
     * Instance of {@link OI}. No subsystem should require this. However, you
     * can read button values from it.
     */
    public static OI oi;
    /**
     * Auto chooser on the smart DS
     */
    SendableChooser<Command> chooser;
    /**
     * Chosen command
     */
    Command autoCmd;
    /**
     * The gyro board on the RoboRIO.
     */
    public static AHRS ahrs;
    /**
     * The Limelight vision system camera.
     */
    public static Limelight limelight;
    UsbCamera camera_0;

    /**
     * Instance of {@link Drivetrain}.
     */
    public static Drivetrain driveTrain;
    public static Rollerclaw rollerClaw;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        ahrs = new AHRS(SPI.Port.kMXP);
        limelight = new Limelight();

        driveTrain = new Drivetrain();
        rollerClaw = new Rollerclaw();

        oi = new OI();

        chooser = new SendableChooser<>();
        chooser.addObject("Reflective chaser", new ReflectivePursuit());
        chooser.addObject("Cube chaser", new CubePursuit());
        System.out.println(chooser.getSelected());
        SmartDashboard.putData("auto selection", chooser);

        camera_0 = CameraServer.getInstance().startAutomaticCapture("flaming bagpipes", 0);
    }

    @Override
    public void autonomousInit() {
        fieldData = DriverStation.getInstance().getGameSpecificMessage();
        autoCmd = chooser.getSelected();
        if (autoCmd != null) {
            autoCmd.start();
        } else {
            System.out.println("Defaulting to reflective pursuit");
            autoCmd = new ReflectivePursuit();
            autoCmd.start();
        }
        camera_0.setExposureManual(20);
    }

    @Override
    public void autonomousPeriodic() {
        if (autoCmd != null) {
            Scheduler.getInstance().run();
        }
    }

    @Override
    public void teleopInit() {
        limelight.setLEDs(Limelight.LIMELIGHT_LED_OFF);
        limelight.setPipeline(1);
        if (autoCmd != null && autoCmd.isRunning()) {
            autoCmd.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        limelight.setLEDs(Limelight.LIMELIGHT_LED_OFF);
    }
}
