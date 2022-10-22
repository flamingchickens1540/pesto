package org.team1540.robotTemplate.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.Vector2d;

public class Limelight {
    private static final double HORIZONTAL_FOV = Math.toRadians(59.6);
    private static final double VERTICAL_FOV = Math.toRadians(49.7);
    private static final Vector2d CAM_RESOLUTION = new Vector2d(320, 240);
    private final NetworkTable limelightTable;

    /**
     * Constructs a new limelight interface with the default hostname.
     *
     * @param name hostname of the limelight
     */
    public Limelight(String name) {
        limelightTable = NetworkTableInstance.getDefault().getTable(name);
        setLeds(LEDMode.OFF);
    }

    public NetworkTable getNetworkTable() {
        return limelightTable;
    }

    public double getHorizontalFov() {
        return HORIZONTAL_FOV;
    }

    public double getVerticalFov() {
        return VERTICAL_FOV;
    }

    public Vector2d getResolution() {
        return CAM_RESOLUTION;
    }

    public double getLeds() {
        return limelightTable.getEntry("ledMode").getDouble(-1);
    }
    /**
     * Sets limelight's green LEDs on or off.
     *
     * @param mode the new state of the LEDs
     */
    public void setLeds(int mode) {
        if (getLeds() != mode) {
            limelightTable.getEntry("ledMode").setNumber(mode);
            NetworkTableInstance.getDefault().flush();
        }
    }

    public long getPipeline() {
        return Math.round((double) limelightTable.getEntry("getpipe").getNumber(-1));
    }

    public void setPipeline(double id) {
        if (getPipeline() != id) {
            limelightTable.getEntry("pipeline").setNumber(id);
            NetworkTableInstance.getDefault().flush();
        }
    }

    /**
     * A mode for the limelight's green LEDs.
     * 
     * @see https://docs.limelightvision.io/en/latest/networktables_api.html
     */
    public static final class LEDMode {
        public static final int PIPELINE = 0;
        public static final int OFF = 1;
        public static final int BLINK = 2;
        public static final int ON = 3;
    }
}
