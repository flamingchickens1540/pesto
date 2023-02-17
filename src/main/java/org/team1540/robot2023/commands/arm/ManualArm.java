package org.team1540.robot2023.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ManualArm extends CommandBase {
    Arm arm;
    CommandXboxController controller;

    public ManualArm(Arm arm, CommandXboxController controller){
        this.arm = arm;
        this.controller = controller;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setManualControl(true);
    }

    @Override
    public void execute() {
        arm.setRotatingSpeed(controller.getLeftY()); // TODO: 2/11/2023 Check angles here
        // TODO: 2/7/2023 Make sure nothing bad happens from repetitive reversing
        //if(arm.getMaxExtension() < arm.getArmState().getExtension()) arm.setExtendingSpeed(-0.7);
        //else if(arm.getMaxExtension() - arm.getArmState().getExtension() < 5) arm.setExtendingSpeed(0); // TODO: 2/11/2023 Make this use the limit switches
        //else arm.setExtendingSpeed(controller.getRightTriggerAxis() - controller.getLeftTriggerAxis());
        arm.setExtendingSpeed(0.25*(controller.getRightTriggerAxis() - controller.getLeftTriggerAxis()));
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopAll();
        arm.setManualControl(false);
    }
}
