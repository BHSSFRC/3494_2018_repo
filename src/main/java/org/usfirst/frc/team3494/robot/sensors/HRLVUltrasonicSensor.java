package org.usfirst.frc.team3494.robot.sensors;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SensorBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * Class to read HRLV-MaxSonar-EZ ultrasonic sensors (specifically MB1013 but can probably support others.)
 * Reads data back from analog input.
 */
public class HRLVUltrasonicSensor extends SensorBase {
    private AnalogInput ai;
    private static double VOLTS_PER_MM = 5 / 5120;

    /**
     * Constructor.
     *
     * @param inputPin The analog I/O pin the sensor is connected to.
     */
    public HRLVUltrasonicSensor(int inputPin) {
        this(new AnalogInput(inputPin));
    }

    /**
     * Constructor.
     *
     * @param inputPin The analog I/O pin the sensor is connected to.
     */
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

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Ultrasonic (analog)");
        builder.addDoubleProperty("Distance (mm)", this::getDistance, null);
    }
}
