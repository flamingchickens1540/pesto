package org.team1540.robot2023.commands.arm;

import org.team1540.robot2023.utils.AverageFilter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExtensionCommand extends CommandBase {
    private final Arm arm;
    private final double targetExtension;
    private final AverageFilter average = new AverageFilter(10);
    private final double threshold = 0.25;

    public ExtensionCommand(Arm arm, double targetExtension) {
        this.arm = arm;
        this.targetExtension = targetExtension;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setExtension(targetExtension);
    }

    @Override
    public void execute() {
        average.add(Math.abs(targetExtension - arm.getArmState().getExtension()));
        arm.setRotation(arm.getArmState().getRotation2d());
    }

    @Override
    public boolean isFinished() {
        return average.getAverage() < threshold;
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopAll();
    }
}