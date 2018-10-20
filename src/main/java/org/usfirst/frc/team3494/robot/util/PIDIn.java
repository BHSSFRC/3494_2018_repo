package org.usfirst.frc.team3494.robot.util;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class PIDIn implements PIDSource {
    private PIDSourceType pidSourceType;
    private FunctionalDoubleManager source;
    public PIDIn(PIDSourceType pidnew, FunctionalDoubleManager source)
    {
        this.pidSourceType = pidnew;
        this.source = source;
    }


    @Override
    public void setPIDSourceType(PIDSourceType pidSourceType) {
        pidSourceType = pidSourceType;
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return pidSourceType;
    }

    @Override
    public double pidGet() {
        return source.get();
    }
}
