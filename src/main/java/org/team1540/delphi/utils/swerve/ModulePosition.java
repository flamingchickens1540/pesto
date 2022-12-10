package org.team1540.delphi.utils.swerve;

/**
 * Represents a corner of the robot. Used for calculating encoder offsets
 */
public enum ModulePosition {
    FRONT_LEFT(8.0, "Front Left"),
    FRONT_RIGHT(329.589, "Front Right"),
    REAR_LEFT(80.33, "Rear Left"),
    REAR_RIGHT(1.4941, "Rear Right");
    private final double offset;
    public final String label;
    ModulePosition(double offset, String label) {
        this.offset = offset;
        this.label = label;
    }
    /**
     * Calculates the corner's offset from the module's offset in the front left corner
     *
     * @return the offset for this corner (in degrees)
     */
    public double get() {
        return offset;
    }
}