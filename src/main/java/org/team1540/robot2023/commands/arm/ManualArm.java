package org.team1540.robot2023.commands.arm;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team1540.robot2023.Constants.ArmConstants;

public class ManualArm extends CommandBase {
    private final SlewRateLimiter slewRateLimiter = new SlewRateLimiter(1,-1,0);
    private final Arm arm;
    private final CommandXboxController controller;
    private final double deadzone = 0.1;
    private boolean isHoldingRotation;
    private boolean isHoldingExtension;
    private boolean startedManualExtension;
    private boolean startedManualPivot;

    public ManualArm(Arm arm, CommandXboxController controller){
        this.arm = arm;
        this.controller = controller;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        isHoldingRotation = false;
        isHoldingExtension = false;
        startedManualExtension = false;
        startedManualPivot = false;
    }

    @Override
    public void execute() {
        double pivotInput = -controller.getLeftY();
        if (!startedManualPivot && Math.abs(pivotInput) >= deadzone) startedManualPivot = true;
        if (startedManualPivot) {
            if (Math.abs(pivotInput) <= deadzone && !isHoldingRotation) {
                isHoldingRotation = true;
                arm.setRotation(arm.getArmState().getRotation2d());
            } else if (Math.abs(pivotInput) >= deadzone) {
                isHoldingRotation = false;
                arm.setRotatingSpeed(Math.pow(pivotInput, 3) * (1 + ((-1 / ArmConstants.TELESCOPE_FORWARD_LIMIT) * pivotInput))); // TODO: 2/11/2023 Check angles here
            }
        }
        double extensionInput = controller.getLeftTriggerAxis() - controller.getRightTriggerAxis();
        if (!startedManualExtension && Math.abs(extensionInput) >= deadzone) startedManualExtension = true;
        double limitedExtensionInput = Math.abs(extensionInput) < deadzone ? 0 : slewRateLimiter.calculate(extensionInput);
        if (startedManualExtension) {
            if (Math.abs(limitedExtensionInput) <= deadzone && !isHoldingExtension) {
                isHoldingExtension = true;
                arm.setExtension(arm.getArmState().getExtension());
            } else if (Math.abs(limitedExtensionInput) >= deadzone) {
                isHoldingExtension = false;
                arm.setExtendingSpeed(limitedExtensionInput);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopAll();
    }
}