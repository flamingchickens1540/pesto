package org.team1540.delphi.utils.swerve;

/**
 * Represents a corner of the robot. Used for calculating encoder offsets
 */
public enum ModulePosition {
    FRONT_LEFT(0, "Front Left"),
    FRONT_RIGHT(90, "Front Right"),
    REAR_LEFT(270, "Rear Left"),
    REAR_RIGHT(180, "Rear Right");
    private final double offset;
    public final String label;
    ModulePosition(double offset, String label) {
        this.offset = offset;
        this.label = label;
    }
    /**
     * Calculates the corner's offset from the module's offset in the front left corner
     * @param moduleOffset the front left corner's offset
     * @return the offset for this corner (in degrees)
     */
    public double getOffset(double moduleOffset) {
        return (moduleOffset-this.offset)%360;
    }
}