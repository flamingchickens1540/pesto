package org.team1540.robot2023.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import org.team1540.lib.math.Conversions;
import org.team1540.robot2023.commands.arm.Arm;

public class ArmState {
    private final double extension;
    private final Rotation2d angle;

    private ArmState(double extension, Rotation2d rotation) {
        this.extension = extension;
        this.angle = rotation;
    }

    public static ArmState fromCartesian(double x, double y) {
        return new ArmState(Math.hypot(x, y), Conversions.cartesianToActual(Rotation2d.fromRadians(Math.atan2(y, x))));
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
        return extension * Math.cos(Conversions.actualToCartesian(angle).getRadians());
    }

    public double getY() {
        return extension * Math.sin(Conversions.actualToCartesian(angle).getRadians());
    }

    @Override
    public boolean equals(Object obj) {
        if(obj.getClass() != ArmState.class){
            return false;
        }
        return extension == ((ArmState)obj).getExtension() && angle.equals(((ArmState)obj).getRotation2d());
    }
}
