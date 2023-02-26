package org.team1540.robot2023.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2023.utils.AverageFilter;

public class PivotCommandTwo extends CommandBase {
    private final Pivot pivot;
    private final Rotation2d targetAngle;
    private final AverageFilter average = new AverageFilter(20);
    private final double threshold = 0.5;

    public PivotCommandTwo(Pivot pivot, Rotation2d targetAngle) {
        this.pivot = pivot;
        this.targetAngle = targetAngle;
        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        pivot.setRotation(targetAngle);
        average.clear();
    }

    @Override
    public void execute() {
        average.add(Math.abs(targetAngle.getDegrees() - pivot.getRotation2d().getDegrees()));
    }

    @Override
    public boolean isFinished() {
        return average.getAverage() < threshold && Math.abs(pivot.getRotation2d().getDegrees() - targetAngle.getDegrees()) < threshold;
    }

    @Override
    public void end(boolean interrupted) {
        pivot.setRotatingSpeed(0);
    }
}