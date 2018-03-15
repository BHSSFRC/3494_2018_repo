package org.usfirst.frc.team3494.robot.commands.auto;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Class for automatic auto routine generation. Executes a list of commands in order and sequentially.
 */
public class DynamicAutoCommand extends CommandGroup {
    /**
     * Constructor.
     *
     * @param commands A list of commands to execute, in order.
     */
    public DynamicAutoCommand(Command[] commands) {
        for (Command c : commands) {
            this.addSequential(c);
        }
    }
}
