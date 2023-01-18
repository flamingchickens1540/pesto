package org.team1540.robot2023.utils.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Represents a corner of the robot. Used for calculating encoder offsets
 */
public enum ModuleCorner {
    FRONT_LEFT(0, "Front Left"),
    FRONT_RIGHT(90, "Front Right"),
    REAR_LEFT(180, "Rear Left"),
    REAR_RIGHT(270, "Rear Right");
    private final double offset;
    public final String label;
    ModuleCorner(double offset, String label) {
        this.offset = offset;
        this.label = label;
    }
    /**
     * Transforms a module's front-left offset to the offset for this corner
     *
     * @return the offset for this corner (in degrees)
     */
    public double transform(int moduleID) {
        return (ModuleMagnetOffset.get(moduleID)+offset)%360;
    }

    /**
     * Transforms a module's front-left offset to the offset for this corner and returns a Rotation2D
     *
     * @return the offset for this corner
     */
    public Rotation2d asRotation2d(int moduleID) {
        return Rotation2d.fromDegrees(transform(moduleID));
    }
}