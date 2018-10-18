package org.usfirst.frc.team3494.robot.commands.auto.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.usfirst.frc.team3494.robot.RobotMap;
import org.usfirst.frc.team3494.robot.commands.auto.AutoInitial;
import org.usfirst.frc.team3494.robot.commands.auto.drive.DistanceDrive;
import org.usfirst.frc.team3494.robot.commands.auto.rollerclaw.RemoveCube;


public class SideAuto extends CommandGroup {

    public SideAuto() {
        addSequential(new AutoInitial());
        addSequential(new DistanceDrive(RobotMap.SIDE_AUTO_DISTANCE_FEET, false));
        addSequential(new RemoveCube());
    }
}
