package org.team1540.robot2023.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExtensionPID extends CommandBase {
    private final Arm arm;
    private final double targetExtension;

    public ExtensionPID(Arm arm, double targetExtension) {
        this.arm = arm;
        this.targetExtension = targetExtension;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setExtensionSetPoint(targetExtension);
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopAll();
    }
}