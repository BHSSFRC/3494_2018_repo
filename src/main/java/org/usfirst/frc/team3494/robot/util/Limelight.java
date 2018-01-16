package org.usfirst.frc.team3494.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    public static final int LIMELIGHT_LED_ON = 0;
    public static final int LIMELIGHT_LED_OFF = 1;
    public static final int LIMELIGHT_LED_BLINK = 2; // no

    private NetworkTable table;

    public Limelight() {
        this("limelight");
    }

    public Limelight(String name) {
        table = NetworkTableInstance.getDefault().getTable(name);
    }

    public void setPipeline(int pipeline) {
        this.table.getEntry("pipeline").setNumber(pipeline);
    }

    public int getPipeline() {
        return this.table.getEntry("pipeline").getNumber(0).intValue();
    }

    public void setLEDs(int state) {
        this.table.getEntry("ledMode").setNumber(state);
    }

    public double getXDistance() {
        return this.table.getEntry("tx").getDouble(0);
    }

    public double getYDistance() {
        return this.table.getEntry("ty").getDouble(0);
    }

    public NetworkTable getTable() {
        return table;
    }
}
