package org.team1540.delphi.utils.swerve;

public enum ModuleOffset {
    
    MODULE1(-239.9414),
    MODULE2(-181.406),
    MODULE3(-169.717),
    MODULE4(-8.174);
    private double offset;
    private ModuleOffset(double offset) {
        this.offset = offset;
    }
    public double getAs(ModulePosition position) {
        return position.getOffset(this.offset);
    }
}
