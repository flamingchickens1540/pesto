package org.team1540.robot2023.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;

public class CalculationTesting {
    public static void main(String[] args) {
        double x = 37;
        double y = 14;
        ArmState armState;
//        armState = ArmState.fromCartesian(x, y);
        armState = ArmState.fromRotationExtension(Rotation2d.fromDegrees(90), 14);
//        System.out.println(Rotation2d.fromRadians(Math.atan2(x, y)));
//        System.out.println(Math.atan2(y, x));
        System.out.println("Extension: " + armState.getExtension());
        System.out.println("Actual Rotation: " + armState.getRotation2d().getDegrees());
        System.out.println("Cartesian Rotation: " + ArmState.actualToCartesian(armState.getRotation2d()).getDegrees());
        System.out.println("X: " + armState.getX());
        System.out.println("Y: " + armState.getY());

    }
}
