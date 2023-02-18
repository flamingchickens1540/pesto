package org.team1540.robot2023.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PivotPID extends CommandBase {
    private final Arm arm;
    private final Rotation2d targetAngle;

    public PivotPID(Arm arm, Rotation2d targetAngle) {
        this.arm = arm;
        this.targetAngle = targetAngle;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setRotation(targetAngle);
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopAll();
    }
}