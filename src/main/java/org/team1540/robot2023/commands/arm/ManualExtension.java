package org.team1540.robot2023.commands.arm;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ManualExtension extends CommandBase {
    private final SlewRateLimiter slewRateLimiter = new SlewRateLimiter(1,-1.5,0);
    private final Arm arm;
    private final CommandXboxController controller;

    public ManualExtension(Arm arm, CommandXboxController controller){
        this.arm = arm;
        this.controller = controller;
        addRequirements(arm.telescope);
    }

    @Override
    public void execute() {
        // TODO: 2/7/2023 Make sure nothing bad happens from repetitive reversing
        //if(arm.getMaxExtension() < arm.getArmState().getExtension()) arm.setExtendingSpeed(-0.7);
        //else if(arm.getMaxExtension() - arm.getArmState().getExtension() < 5) arm.setExtendingSpeed(0); // TODO: 2/11/2023 Make this use the limit switches
        //else arm.setExtendingSpeed(controller.getRightTriggerAxis() - controller.getLeftTriggerAxis());
        arm.telescope.setExtendingSpeed(slewRateLimiter.calculate(controller.getLeftTriggerAxis() - controller.getRightTriggerAxis()));
        
    }

    @Override
    public void end(boolean interrupted) {
        arm.telescope.stop();
    }
}