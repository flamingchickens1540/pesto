package org.team1540.robot2023.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RetractExtension extends CommandBase {
    private final Telescope telescope;

    public RetractExtension(Arm arm) {
        this.telescope = arm.telescope;
        addRequirements(arm.telescope);
    }

    @Override
    public void initialize() {
        telescope.setExtendingSpeed(-0.7);
    }

    @Override
    public boolean isFinished() {
        return telescope.getLimitSwitch();
    }

    @Override
    public void end(boolean interrupted) {
        telescope.setExtendingSpeed(0);
    }
}
