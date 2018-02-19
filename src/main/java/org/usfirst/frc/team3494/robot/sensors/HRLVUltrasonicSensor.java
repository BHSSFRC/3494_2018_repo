package org.usfirst.frc.team3494.robot.sensors;

import edu.wpi.first.wpilibj.AnalogInput;

/**
 * Class to read HRLV-MaxSonar-EZ ultrasonic sensors (specifically MB1013 but can probably support others.)
 * Reads data back from analog input.
 */
public class HRLVUltrasonicSensor {
    private AnalogInput ai;
    private static double VOLTS_PER_MM = 5 / 5120;

    public HRLVUltrasonicSensor(int inputPin) {
        this(new AnalogInput(inputPin));
    }

    public HRLVUltrasonicSensor(AnalogInput inputPin) {
        ai = inputPin;
    }

    /**
     * Returns the range found by the sensor in millimeters. May "float" slightly because analog is analog.
     *
     * @return The distance found in mm.
     */
    public double getDistance() {
        return ai.getVoltage() / HRLVUltrasonicSensor.VOLTS_PER_MM;
    }
}
