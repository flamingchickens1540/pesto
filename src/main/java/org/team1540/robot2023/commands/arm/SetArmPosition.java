package org.team1540.robot2023.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetArmPosition extends CommandBase {
    private final Arm arm;
    private final double angle;
    private final double extension;

    public SetArmPosition(Arm arm, double angle, double extension){
        this.arm = arm;
        this.angle = angle;
        this.extension = extension;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setAngleRadians(angle);
    }

    @Override
    public void execute() {
        arm.setExtension(Math.min(extension, arm.getMaxExtension()));
    }

    @Override
    public boolean isFinished() {
        return Math.abs(arm.getAngleRadians() - angle) < 0.01 && Math.abs(arm.getExtension() - extension) < 0.01;
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopAll();
    }
}
