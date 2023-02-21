package org.team1540.robot2023.commands.arm;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team1540.robot2023.Constants.ArmConstants;

public class ManualArm extends CommandBase {
    private final SlewRateLimiter slewRateLimiter = new SlewRateLimiter(1,-1.5,0);
    private final Arm arm;
    private final CommandXboxController controller;
    private final double deadzone = 0.1;
    private boolean isHolding;

    public ManualArm(Arm arm, CommandXboxController controller){
        this.arm = arm;
        this.controller = controller;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setManualControl(true);
        isHolding = false;
    }

    @Override
    public void execute() {
        double pivotInput = -controller.getLeftY();
        if (Math.abs(pivotInput) <= deadzone && !isHolding) {
            isHolding = true;
            arm.setRotation(arm.getArmState().getRotation2d());
        } else if (Math.abs(pivotInput) > deadzone) {
            isHolding = false;
            arm.setRotatingSpeed(Math.pow(pivotInput, 3) * (1 + ((-1/ArmConstants.TELESCOPE_FORWARD_LIMIT)*pivotInput))); // TODO: 2/11/2023 Check angles here
        }
        // TODO: 2/7/2023 Make sure nothing bad happens from repetitive reversing
        //if(arm.getMaxExtension() < arm.getArmState().getExtension()) arm.setExtendingSpeed(-0.7);
        //else if(arm.getMaxExtension() - arm.getArmState().getExtension() < 5) arm.setExtendingSpeed(0); // TODO: 2/11/2023 Make this use the limit switches
        //else arm.setExtendingSpeed(controller.getRightTriggerAxis() - controller.getLeftTriggerAxis());
        arm.setExtendingSpeed(slewRateLimiter.calculate(controller.getLeftTriggerAxis() - controller.getRightTriggerAxis()));
        
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopAll();
        arm.setManualControl(false);
    }
}