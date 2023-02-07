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
        return new ArmState(Math.hypot(x, y), cartesianToActual(Rotation2d.fromRadians(Math.atan2(y, x))));
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
        return extension * Math.cos(actualToCartesian(angle).getRadians());
    }

    public double getY() {
        return extension * Math.sin(actualToCartesian(angle).getRadians());
    }

    public static Rotation2d cartesianToActual(Rotation2d angle) {
        double theta = angle.getDegrees();
        return Rotation2d.fromDegrees(theta - 90 < -180 ? theta + 270 : theta - 90);
    }

    public static Rotation2d actualToCartesian(Rotation2d angle) {
        double theta = angle.getDegrees();
        return Rotation2d.fromDegrees(theta + 90 > 180 ? theta - 270 : theta + 90);
    }
}
