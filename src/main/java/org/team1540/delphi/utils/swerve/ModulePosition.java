package org.team1540.delphi.utils.swerve;

public enum ModulePosition {
    FRONT_LEFT(0),
    FRONT_RIGHT(90),
    REAR_LEFT(270),
    REAR_RIGHT(180);
    private double offset;
    ModulePosition(double offset) {
        this.offset = offset;
    }
    public double getOffset(double moduleOffset) {
        return (moduleOffset-this.offset)%360;
    }
}