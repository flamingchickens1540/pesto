package org.team1540.robot2023.utils.swerve;


/**
 * Represents a swerve module's magnet offset. Measured by aligning the wheel so that it would be facing left if the module was in the top left corner and finding the value that would make it zero.
 */
public enum ModuleOffset {
    MODULE1(232.139),
    MODULE2(337.061),
    MODULE3(231.680),
    MODULE4(333.809);
    private double offset;
    private ModuleOffset(double offset) {
        this.offset = offset;
    }

    public double get() {return offset;}
}
