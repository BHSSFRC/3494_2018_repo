package org.usfirst.frc.team3494.robot.util;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class PIDIn implements PIDSource
{
        private FunctionalDoubleManager source;
        private PIDSourceType pidSourceType;
        public PIDIn(FunctionalDoubleManager source, PIDSourceType pidSource)
        {
            this.pidSourceType = pidSource;
            this.source = source;
        }

        @Override
        public void setPIDSourceType(PIDSourceType pidSourceType){
            this.pidSourceType = pidSourceType;
        }
        @Override
        public PIDSourceType getPIDSourceType(){
            return this.pidSourceType;
        }
        @Override
        public double pidGet(){
            return this.source.get();
        }
}

