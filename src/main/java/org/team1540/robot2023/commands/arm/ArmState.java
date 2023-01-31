package org.team1540.robot2023.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;

public class ArmState {
    private final double extension;
    private final Rotation2d angle;

    private ArmState(double extension, Rotation2d rotation) {
        this.extension = extension;
        this.angle = rotation;
    }

    public static ArmState fromCartesian(double x, double y) {
        return new ArmState(Math.hypot(x, y), Rotation2d.fromRadians(Math.atan(x / y)));
    }

    public static ArmState fromRotationExtension(Rotation2d rotation, double extension) {
        return new ArmState(extension, Rotation2d.fromDegrees((rotation.getDegrees() % 360)));
    }

    public double getExtension() {
        return extension;
    }

    public Rotation2d getRotation2d() {
        return angle;
    }

    public double getX() {
        return extension * Math.sin(angle.getRadians());
    }

    public double getY() {
        return extension * Math.cos(angle.getRadians());
    }
}
