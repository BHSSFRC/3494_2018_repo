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
import org.usfirst.frc.team3494.robot.commands.auto.PathBuilder;
import org.usfirst.frc.team3494.robot.commands.auto.ReflectivePursuit;
import org.usfirst.frc.team3494.robot.commands.auto.tests.PathTestOne;
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
    /**
     * Instance of {@link Rollerclaw}.
     */
    public static Rollerclaw rollerClaw;

    public static PathBuilder pathBuilder;

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
        pathBuilder = new PathBuilder();

        // pathBuilder.getCenterToRightTraj();
        pathBuilder.getCenterToLeftTraj();

        chooser = new SendableChooser<>();
        chooser.addObject("Reflective chaser", new ReflectivePursuit());
        chooser.addObject("Cube chaser", new CubePursuit());
        chooser.addObject("Path tester", new PathTestOne());
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

        SmartDashboard.putNumber("Left enc", Robot.driveTrain.getCountsLeft_Talon());
        SmartDashboard.putNumber("Right enc", Robot.driveTrain.getCountsRight_Talon());

        SmartDashboard.putNumber("Left speed", Drivetrain.nativeToRPS(Robot.driveTrain.getVelocityLeft()));
        SmartDashboard.putNumber("Left speed wheel revs per sec", Drivetrain.nativeToRPS(Robot.driveTrain.getVelocityLeft()) * 3 / 11.9);
        SmartDashboard.putNumber("Right speed", Drivetrain.nativeToRPS(Robot.driveTrain.getVelocityRight()));
    }

    @Override
    public void teleopInit() {
        limelight.setLEDs(Limelight.LIMELIGHT_LED_OFF);
        limelight.setPipeline(1);
        if (autoCmd != null && autoCmd.isRunning()) {
            autoCmd.cancel();
        }
        Robot.driveTrain.resetEncoders();
        SmartDashboard.putNumber("Left speed wheel revs per sec", 0);
    }

    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();

        SmartDashboard.putNumber("Left enc", Robot.driveTrain.getCountsLeft_Talon());
        SmartDashboard.putNumber("Right enc", Robot.driveTrain.getCountsRight_Talon());

        SmartDashboard.putNumber("Left wheel revolutions", (Robot.driveTrain.getCountsLeft_Talon() / 4) * 3 / 11.9 / 256);

        SmartDashboard.putNumber("Left speed", Drivetrain.nativeToRPS(Robot.driveTrain.getVelocityLeft()));
        SmartDashboard.putNumber("Left speed wheel revs per sec", Drivetrain.nativeToRPS(Robot.driveTrain.getVelocityLeft()) * 3 / 11.9);
        SmartDashboard.putNumber("Right speed", Drivetrain.nativeToRPS(Robot.driveTrain.getVelocityRight()));
    }

    @Override
    public void disabledInit() {
        limelight.setLEDs(Limelight.LIMELIGHT_LED_OFF);
    }

    /**
     * Convert a number of encoder counts to a distance in meters.
     *
     * @param counts The number of counts to be converted.
     * @return The distance in meters traveled in the given number of counts.
     */
    public static double countsToMeters(double counts) {
        return counts / RobotMap.COUNTS_PER_METER;
    }

    /**
     * Convert a distance in meters to a number of encoder counts.
     *
     * @param meters The number of meters to convert to counts.
     * @return The number of counts in the given distance.
     */
    public static double metersToCounts(double meters) {
        return meters * RobotMap.COUNTS_PER_METER;
    }

    /**
     * Convert a distance in meters to a number of encoder edges.
     *
     * @param meters The number of meters to convert to edges.
     * @return The number of edges in the given distance.
     */
    public static double metersToEdges(double meters) {
        return Robot.metersToCounts(meters) * 4;
    }
}
