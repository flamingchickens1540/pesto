package org.team1540.robot2023.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2023.Constants;
import org.team1540.robot2023.utils.RollingAverage;

public class SetExtension extends CommandBase {
    private final double threshold = 10;
    private final RollingAverage rollingAverage = new RollingAverage(10);

    private final Arm arm;

    private final double extension;

    public SetExtension(Arm arm, double extension) {
        this.arm = arm;
        this.extension = extension;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setExtensionSetPoint(extension);
    }

    @Override
    public void execute() {
        rollingAverage.add(arm.getExtension() - extension);
    }

    @Override
    public boolean isFinished() {
        return rollingAverage.getAverageAbs() < threshold;
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopAll();
    }
}
