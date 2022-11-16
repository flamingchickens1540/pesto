package org.team1540.delphi.utils.swerve;


/**
 * Represents a swerve module's magnet offset. Measured by aligning the wheel so that it would be facing left if the module was in the top left corner and finding the value that would make it zero.
 */
public enum ModuleOffset {
    MODULE1(-239.9414),
    MODULE2(-181.406),
    MODULE3(-169.717),
    MODULE4(-8.174);
    private double offset;
    private ModuleOffset(double offset) {
        this.offset = offset;
    }

    /**
     * Gets the offset for a given corner
     * @param position which corner of the robot the module is in
     * @return The offset (in degrees)
     */
    public double getAs(ModulePosition position) {
        return position.getOffset(this.offset);
    }
}
