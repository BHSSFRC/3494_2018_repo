package org.usfirst.frc.team3494.robot.util;

import edu.wpi.first.wpilibj.PIDOutput;

public class PIDOut implements PIDOutput {

    public double output;

    @Override
    public void pidWrite(double output) {

        this.output = output;
    }
}
