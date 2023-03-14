package org.team1540.robot2023.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2023.utils.ArmState;
import org.team1540.robot2023.utils.AverageFilter;

public class PivotCommand extends CommandBase {
    private final Arm arm;
    private final Rotation2d targetAngle;
    private final AverageFilter average = new AverageFilter(5);
    private final double threshold = 0.5;


    public PivotCommand(Arm arm, ArmState target) {
        this(arm, target.getRotation2d());
    }
    public PivotCommand(Arm arm, Rotation2d targetAngle) {
        this.arm = arm;
        this.targetAngle = targetAngle;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setRotation(targetAngle);
        average.clear();
        arm.setExtension(arm.getArmState().getExtension());
    }

    @Override
    public void execute() {
        average.add(Math.abs(targetAngle.getDegrees() - arm.getArmState().getRotation2d().getDegrees()));
    }

    @Override
    public boolean isFinished() {
//        return false;
        return average.getAverage() < threshold && Math.abs(arm.getArmState().getRotation2d().getDegrees() - targetAngle.getDegrees()) < threshold;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) arm.holdPivot();
    }
}