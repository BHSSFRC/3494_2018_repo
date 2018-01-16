package org.usfirst.frc.team3494.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.SPI;
import org.usfirst.frc.team3494.robot.subsystems.Drivetrain;


public class Robot extends IterativeRobot {
    private String fieldData;
    /**
     * Instance of {@link OI}. No subsystem should require this. However, you
     * can read button values from it.
     */
    public static OI oi;
    public static Drivetrain driveTrain;
    public static AHRS ahrs;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        oi = new OI();
        ahrs = new AHRS(SPI.Port.kMXP);

        driveTrain = new Drivetrain();
    }

    @Override
    public void autonomousInit() {
        fieldData = DriverStation.getInstance().getGameSpecificMessage();
    }
}
