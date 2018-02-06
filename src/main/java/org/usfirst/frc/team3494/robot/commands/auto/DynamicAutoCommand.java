package org.usfirst.frc.team3494.robot.commands.auto;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class DynamicAutoCommand extends CommandGroup {
    public DynamicAutoCommand(Command[] commands) {
        for (Command c : commands) {
            this.addSequential(c);
        }
    }
}
