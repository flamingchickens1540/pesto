package org.team1540.delphi.utils.swerve;

/**
 * Represents a corner of the robot. Used for calculating encoder offsets
 */
public enum ModulePosition {
    FRONT_LEFT(0),
    FRONT_RIGHT(90),
    REAR_LEFT(270),
    REAR_RIGHT(180);
    private double offset;
    ModulePosition(double offset) {
        this.offset = offset;
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