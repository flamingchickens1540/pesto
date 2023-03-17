package org.team1540.robot2023.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2023.utils.ArmState;
import org.team1540.robot2023.utils.AverageFilter;

public class PivotCommand extends CommandBase {
    private final Arm arm;
    private final Rotation2d targetAngle;
    private boolean shouldZero;
    private final AverageFilter average = new AverageFilter(5);
    private final double threshold = 0.5;
    private long endTime;


    public PivotCommand(Arm arm, ArmState target) {
        this(arm, target.getRotation2d());
    }

    public PivotCommand(Arm arm, Rotation2d targetAngle) {
        this(arm, targetAngle, true);
    }
    public PivotCommand(Arm arm, Rotation2d targetAngle, boolean shouldZero) {
        this.arm = arm;
        this.targetAngle = targetAngle;
        this.shouldZero = shouldZero;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        if (shouldZero) {
            arm.resetToEncoder();
        }
        arm.setRotation(targetAngle);
        average.clear();
        arm.setExtension(arm.getArmState().getExtension());
        endTime = (long) arm.timeToRotation(targetAngle) + System.currentTimeMillis() + 250;
    }

    @Override
    public void execute() {
        average.add(Math.abs(targetAngle.getDegrees() - arm.getArmState().getRotation2d().getDegrees()));
    }

    @Override
    public boolean isFinished() {
        return (average.getAverage() < threshold && Math.abs(arm.getArmState().getRotation2d().getDegrees() - targetAngle.getDegrees()) < threshold)
                || System.currentTimeMillis() >= endTime;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) arm.setRotatingSpeed(0);
//        if (shouldZero) {
//            arm.resetToEncoder();
//        }
    }
}