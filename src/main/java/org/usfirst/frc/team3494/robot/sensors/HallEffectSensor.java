package org.usfirst.frc.team3494.robot.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Class to represent Hall Effect sensors (magnet detectors.)
 */
public class HallEffectSensor {
    private DigitalInput source;

    /**
     * Creates a new HallEffectSensor.
     *
     * @param channel The DIO channel the sensor is connected to.
     */
    public HallEffectSensor(int channel) {
        this.source = new DigitalInput(channel);
    }

    /**
     * Returns true if there's a magnet nearby.
     *
     * @return true if there's a magnet nearby.
     */
    public boolean isActive() {
        return !this.source.get();
    }
}
