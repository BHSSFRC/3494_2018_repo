package org.usfirst.frc.team3494.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import org.usfirst.frc.team3494.robot.subsystems.Drivetrain;


public class Robot extends IterativeRobot {
    /**
     * Instance of {@link OI}. No subsystem should require this. However, you
     * can read button values from it.
     */
    public static OI oi;
    public static Drivetrain driveTrain;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        oi = new OI();
        driveTrain = new Drivetrain();
    }
}
