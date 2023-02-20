package org.team1540.robot2023.commands.arm;

import org.team1540.robot2023.utils.AverageFilter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PivotCommand extends CommandBase {
    private final Arm arm;
    private final Rotation2d targetAngle;
    private final AverageFilter average = new AverageFilter(10);
    private final double threshold = 2.4;

    public PivotCommand(Arm arm, Rotation2d targetAngle) {
        this.arm = arm;
        this.targetAngle = targetAngle;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setRotation(targetAngle);
    }

    @Override
    public void execute() {
        average.add(Math.abs(targetAngle.getDegrees() - arm.getArmState().getRotation2d().getDegrees()));
    }

    @Override
    public boolean isFinished() {
        return average.getAverage() < threshold;
    }

    @Override
    public void end(boolean interrupted) {
        arm.setRotatingSpeed(0);
    }
}