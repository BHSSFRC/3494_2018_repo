package org.usfirst.frc.team3494.robot.commands.auto.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.usfirst.frc.team3494.robot.RobotMap;
import org.usfirst.frc.team3494.robot.commands.auto.AutoInitial;
import org.usfirst.frc.team3494.robot.commands.auto.drive.DistanceDrive;


public class notHotSideAuto extends CommandGroup {

    public notHotSideAuto() {
        addSequential(new AutoInitial());
        addSequential(new DistanceDrive(RobotMap.SIDE_AUTO_DISTANCE_FEET, false));
    }
}
