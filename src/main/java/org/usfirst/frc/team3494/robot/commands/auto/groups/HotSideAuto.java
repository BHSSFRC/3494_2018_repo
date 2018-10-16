package org.usfirst.frc.team3494.robot.commands.auto.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.usfirst.frc.team3494.robot.RobotMap;
import org.usfirst.frc.team3494.robot.commands.auto.AutoInitial;
import org.usfirst.frc.team3494.robot.commands.auto.drive.DistanceDrive;
import org.usfirst.frc.team3494.robot.commands.auto.rollerclaw.RemoveCube;


public class HotSideAuto extends CommandGroup {

    public HotSideAuto() {
        addSequential(new AutoInitial());
        addSequential(new DistanceDrive(RobotMap.Field.DISTANCE_TO_SWITCH, false));
        addSequential(new RemoveCube());
    }
}
