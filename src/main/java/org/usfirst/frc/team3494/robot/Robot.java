package org.usfirst.frc.team3494.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Trajectory;
import org.usfirst.frc.team3494.robot.commands.auto.AutoInitial;
import org.usfirst.frc.team3494.robot.commands.auto.DynamicAutoCommand;
import org.usfirst.frc.team3494.robot.commands.auto.drive.AngleDrive;
import org.usfirst.frc.team3494.robot.commands.auto.drive.DistanceDrive;
import org.usfirst.frc.team3494.robot.commands.auto.drive.ProfileFollower;
import org.usfirst.frc.team3494.robot.commands.auto.drive.TalonProfileFollower;
import org.usfirst.frc.team3494.robot.commands.auto.lift.LiftToHeight;
import org.usfirst.frc.team3494.robot.commands.auto.rollerclaw.RemoveCube;
import org.usfirst.frc.team3494.robot.commands.auto.tests.CubePursuit;
import org.usfirst.frc.team3494.robot.commands.auto.tests.QuickDirtyDrive;
import org.usfirst.frc.team3494.robot.commands.auto.vision.ReflectivePursuit;
import org.usfirst.frc.team3494.robot.sensors.Limelight;
import org.usfirst.frc.team3494.robot.subsystems.*;

import java.util.HashMap;


public class Robot extends IterativeRobot {
    private String fieldData;
    /**
     * A {@link HashMap} of CSV files for {@link ProfileFollower}.
     * Keys are in the form of START+END where START and END are either {@code L} or {@code R},
     * for left and right respectively. (i.e. if START and END are both L the key should be {@code LL}.)
     * <p>
     * Values are string lists representing the location of the appropriate CSV files in the RoboRIO's filesystem.
     */
    private static HashMap<String, String[]> autoFiles;
    /**
     * Instance of {@link OI}. No subsystem should require this. However, you
     * can read button values from it.
     */
    public static OI oi;
    /**
     * Auto chooser on the {@link SmartDashboard}.
     */
    private SendableChooser<Command> chooser;
    /**
     * Chooser for robot position (left, right, center.)
     */
    private SendableChooser<String> positionChooser;
    /**
     * Chosen command for auto.
     */
    private Command autoCmd;
    public static PowerDistributionPanel pdp;
    /**
     * The gyro board on the RoboRIO.
     */
    public static AHRS ahrs;
    /**
     * The Limelight vision system camera.
     */
    public static Limelight limelight;
    /**
     * Instance of {@link Drivetrain}.
     */
    public static Drivetrain driveTrain;
    /**
     * Instance of {@link Rollerclaw}.
     */
    public static Rollerclaw rollerClaw;
    /**
     * Instance of {@link Lights}.
     */
    public static Lights lights;
    /**
     * Instance of {@link Ramps}.
     */
    public static Ramps ramps;
    /**
     * Instance of {@link Lift}.
     */
    public static Lift lift;

    private static Timer timer;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        Robot.initAutoFiles();

        new Compressor().start();

        fieldData = DriverStation.getInstance().getGameSpecificMessage();

        ahrs = new AHRS(SPI.Port.kMXP);
        pdp = new PowerDistributionPanel();
        limelight = new Limelight();

        driveTrain = new Drivetrain();
        rollerClaw = new Rollerclaw();
        lights = new Lights();
        ramps = new Ramps();
        ramps.retract();
        ramps.closeClaw();
        lift = new Lift();

        oi = new OI();
        timer = new Timer();
        timer.reset();
        timer.start();

        CameraServer.getInstance().startAutomaticCapture("Lift Camera", 0);
        CameraServer.getInstance().startAutomaticCapture("Rearview Camera", 1);

        chooser = new SendableChooser<>();
        chooser.addObject("Reflective chaser", new org.usfirst.frc.team3494.robot.commands.auto.tests.ReflectivePursuit(0));
        chooser.addObject("Cube chaser", new CubePursuit());
        chooser.addObject("Auto Init test", new AutoInitial());
        chooser.addObject("Cross baseline", new DistanceDrive(10.0D - (33.0 / 12.0)));
        chooser.addObject("Simpler fully automatic auto", new QuickDirtyDrive());
        chooser.addDefault("Fully automated auto", null);
        SmartDashboard.putData("auto selection", chooser);

        positionChooser = new SendableChooser<>();
        positionChooser.addObject("left", "L");
        positionChooser.addDefault("center", "C");
        positionChooser.addObject("right", "R");
        SmartDashboard.putData("Position chooser", positionChooser);
    }

    @Override
    public void robotPeriodic() {
        // make the logs be quiet
    }

    @Override
    public void autonomousInit() {
        fieldData = DriverStation.getInstance().getGameSpecificMessage();
        System.out.println("Hey FTA! Pay attention! Received field data " + fieldData);
        autoCmd = chooser.getSelected();
        if (autoCmd != null && !(autoCmd instanceof QuickDirtyDrive)) {
            autoCmd.start();
        } else {
            Command[] cmdList;
            String switchSide = String.valueOf(fieldData.charAt(0));
            String startSide = positionChooser.getSelected();
            if (autoCmd == null) {
                System.out.println("Defaulting to fully automatic auto");
                System.out.println(startSide + switchSide);
                // generate appropriate command
                String[] autoFiles = Robot.autoFiles.get(startSide + switchSide);
                if (startSide.equals(switchSide)) {
                    cmdList = new Command[]{
                            new AutoInitial(),
                            new TalonProfileFollower(autoFiles[0], autoFiles[1]),
                            new RemoveCube()
                    };
                } else if (startSide.equals("C")) {
                    cmdList = new Command[]{
                            new AutoInitial(),
                            new TalonProfileFollower(autoFiles[0], autoFiles[1]),
                            new ReflectivePursuit(0),
                            new RemoveCube()
                    };
                } else {
                    System.out.println("or just cross base");
                    cmdList = new Command[]{
                            new AutoInitial(),
                            new DistanceDrive(10.0D - (66.0 / 12.0)) // cross base
                    };
                }
                autoCmd = new DynamicAutoCommand(cmdList);
            } else {
                // worse version of fully automatic auto
                if (!startSide.equals("C")) {
                    if (switchSide.equals(startSide)) {
                        switch (startSide) {
                            case "R":
                                cmdList = new Command[]{
                                        new DistanceDrive(10.0D + (55.5 / 12.0) - (33.0 / 12.0)),
                                        new AngleDrive(-90.0),
                                        new RemoveCube()
                                };
                                break;
                            case "L":
                                cmdList = new Command[]{
                                        new DistanceDrive(10.0D + (55.5 / 12.0) - (33.0 / 12.0)),
                                        new AngleDrive(90.0),
                                        new RemoveCube()
                                };
                                break;
                            default:
                                cmdList = new Command[]{
                                        new DistanceDrive(10.0D - (33.0 / 12.0))
                                };
                        }
                        autoCmd = new DynamicAutoCommand(cmdList);
                    }
                } else {
                    autoCmd = new DistanceDrive(10.0D - (33.0 / 12.0));
                }
            }
            autoCmd.start();
        }
    }

    @Override
    public void autonomousPeriodic() {
        if (autoCmd != null) {
            Scheduler.getInstance().run();
        }
        Robot.putDebugInfo();
    }

    @Override
    public void teleopInit() {
        if (autoCmd != null) {
            autoCmd.cancel();
        }
        limelight.setLEDs(Limelight.LIMELIGHT_LED_OFF);
        limelight.setPipeline(1);
        if (autoCmd != null && autoCmd.isRunning()) {
            autoCmd.cancel();
        }
        Robot.driveTrain.resetEncoders();
    }

    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
        Robot.putDebugInfo();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
        fieldData = DriverStation.getInstance().getGameSpecificMessage();
        Robot.putDebugInfo();
    }

    private static void putDebugInfo() {
        SmartDashboard.putNumber("Angle", Robot.ahrs.getAngle());
        SmartDashboard.putData(Scheduler.getInstance());
    }

    public static Timer getTimer() {
        return timer;
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

    public static double metersToEdges(double meters) {
        return Robot.metersToCounts(meters) * 4;
    }

    public static double feetToMeters(double feet) {
        return feet * 0.3048;
    }

    public static double feetToCounts(double feet) {
        return feet * RobotMap.COUNTS_PER_FOOT;
    }

    public static double feetToEdges(double feet) {
        return Robot.feetToCounts(feet) * 4;
    }

    public static double countsToFeet(double counts) {
        return counts / RobotMap.COUNTS_PER_FOOT;
    }

    public static double[][] pathfinderFormatToTalon(Trajectory t) {
        int i = 0;
        double[][] list = new double[t.length()][3];
        for (Trajectory.Segment s : t.segments) {
            list[i][0] = s.position;
            list[i][1] = s.velocity;
            list[i][2] = s.dt;
            i++;
        }
        return list;
    }

    /**
     * Limit values to a range.
     *
     * @param num   The number to limit to [-bound, bound].
     * @param bound The bounds to use in limiting.
     * @return The limited value.
     */
    public static double limit(double num, double bound) {
        bound = Math.abs(bound);
        if (num > bound) {
            return bound;
        }
        if (num < -bound) {
            return -bound;
        }
        return num;
    }

    /**
     * Returns 0.0 if the given value is within the specified range around zero. The remaining range
     * between the deadband and 1.0 is scaled from 0.0 to 1.0.
     *
     * @param value    value to clip
     * @param deadband range around zero
     * @return Zero if the value is in the deadband, or the value unchanged.
     * @author Worcester Polytechnic Institute
     */
    public static double applyDeadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    /**
     * Sets up {@link Robot#autoFiles} with the paths to the various CSV files.
     */
    private static void initAutoFiles() {
        autoFiles = new HashMap<>();

        autoFiles.put("CL", new String[]{
                "/home/lvuser/paths_finder/center/center2left_left.csv",
                "/home/lvuser/paths_finder/center/center2left_right.csv"
        });
        autoFiles.put("CR", new String[]{
                "/home/lvuser/paths_finder/center/center2right_left.csv",
                "/home/lvuser/paths_finder/center/center2right_right.csv"
        });

        autoFiles.put("LL", new String[]{
                "/home/lvuser/paths_finder/left/left2left_left.csv",
                "/home/lvuser/paths_finder/left/left2left_right.csv"
        });
        autoFiles.put("LR", new String[]{
                "/home/lvuser/paths_finder/left/left2right_left.csv",
                "/home/lvuser/paths_finder/left/left2right_right.csv"
        });

        autoFiles.put("RL", new String[]{
                "/home/lvuser/paths_finder/right/right2left_left.csv",
                "/home/lvuser/paths_finder/right/right2left_right.csv"
        });
        autoFiles.put("RR", new String[]{
                "/home/lvuser/paths_finder/right/right2right_left.csv",
                "/home/lvuser/paths_finder/right/right2right_right.csv"
        });
    }
}
