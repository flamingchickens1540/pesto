package org.team1540.robot2023.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2023.Constants;
import org.team1540.robot2023.utils.AverageFilter;

public class ExtensionCommand extends CommandBase {
    private final Arm arm;
    private final double targetExtension;
    private final AverageFilter average = new AverageFilter(10);
    private static final double threshold = 0.25;

    public ExtensionCommand(Arm arm, double targetExtension) {
        this.arm = arm;
        this.targetExtension = targetExtension;
        addRequirements(arm.telescope);
    }

    @Override
    public void initialize() {
        System.out.println("EXTAENDING START");
        arm.telescope.setExtension(targetExtension);
        average.clear();
    }

    @Override
    public void execute() {
        System.out.println("EXTAENDING");
        average.add(Math.abs(targetExtension - arm.getArmState().getExtension()));
    }

    @Override
    public boolean isFinished() {
        if(targetExtension < Constants.ArmConstants.ARM_BASE_LENGTH && arm.telescope.getLimitSwitch()){
            return true;
        }
        return average.getAverage() < threshold;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("EXTAENDING DONE");
        arm.stopAll();
    }
}