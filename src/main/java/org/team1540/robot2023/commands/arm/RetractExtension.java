package org.team1540.robot2023.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RetractExtension extends CommandBase {
    private final Arm arm;

    public RetractExtension(Arm arm) {
        this.arm = arm;
    }

    @Override
    public void initialize() {
        arm.setExtendingSpeed(-0.1);
    }

    @Override
    public boolean isFinished() {
        return arm.getLimitSwitch();
    }

    @Override
    public void end(boolean interrupted) {
        arm.setExtendingSpeed(0);
    }
}
