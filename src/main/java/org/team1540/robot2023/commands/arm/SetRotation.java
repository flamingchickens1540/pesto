package org.team1540.robot2023.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2023.utils.RollingAverage;

public class SetRotation extends CommandBase {
    private final double threshold = 10;
    private final RollingAverage rollingAverage = new RollingAverage(10);

    private final Arm arm;

    private final double angle;

    public SetRotation(Arm arm, double angle) {
        this.arm = arm;
        this.angle = angle;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setRotation(angle);
    }

    @Override
    public void execute() {
        rollingAverage.add(arm.getRotation2d() - angle);
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
