package org.team1540.robot2023.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2023.utils.RollingAverage;

public class SetRotation extends CommandBase {
    private final double threshold = 10;
    private final RollingAverage rollingAverage = new RollingAverage(10);

    private final Arm arm;

    private final double angle;

    public SetRotation(Arm arm, double x, double y) {
        this.arm = arm;
        this.angle = Math.atan(x / y);
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setAngleRadians(angle);
    }

    @Override
    public void execute() {
        rollingAverage.add(arm.getAngleRadians() - angle);
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
