package org.team1540.robot2023.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2023.Constants;
import org.team1540.robot2023.utils.AverageFilter;

public class ExtensionCommandTwo extends CommandBase {
    private final Telescope telescope;
    private final double targetExtension;
    private final AverageFilter average = new AverageFilter(10);
    private final double threshold = 0.25;

    public ExtensionCommandTwo(Telescope telescope, double targetExtension) {
        this.telescope = telescope;
        this.targetExtension = targetExtension;
        addRequirements(telescope);
    }

    @Override
    public void initialize() {
        telescope.setExtension(targetExtension);
        average.clear();
    }

    @Override
    public void execute() {
        average.add(Math.abs(targetExtension - telescope.getExtension()));
    }

    @Override
    public boolean isFinished() {
        if(targetExtension < Constants.ArmConstants.ARM_BASE_LENGTH && telescope.getLimitSwitch()){
            return true;
        }
        return average.getAverage() < threshold;
    }

    @Override
    public void end(boolean interrupted) {
        telescope.stop();
    }
}