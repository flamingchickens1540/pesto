package org.team1540.robot2023.commands.arm;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualArm extends CommandBase {
    Arm arm;
    XboxController controller;

    public ManualArm(Arm arm, XboxController controller){
        this.arm = arm;
        this.controller = controller;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.setRotatingSpeed(controller.getLeftY());
        if(arm.getMaxExtension() < arm.getArmState().getExtension())arm.setExtendingSpeed(-0.7);
        else arm.setExtendingSpeed(controller.getRightTriggerAxis() - controller.getLeftTriggerAxis());
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopAll();
    }
}
