package org.team1540.robot2023.commands.arm;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team1540.robot2023.Constants.ArmConstants;

public class ManualArm extends CommandBase {
    private final SlewRateLimiter extensionRateLimiter = new SlewRateLimiter(1,-1,0);
    private final Arm arm;
    private final CommandXboxController controller;
    private final double deadzone = 0.1;
    private boolean isHoldingRotation;
    private boolean isHoldingExtension;
    private boolean startedManualExtension;
    private boolean startedManualPivot;

    private double pivotScale = 1;
    private double extensionScale = 1;

    public ManualArm(Arm arm, CommandXboxController controller) {
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
        if(RobotState.isTest()){
            pivotScale = 1;
            extensionScale = 0;
        }
        else {
            pivotScale = 1;
            extensionScale = 1;
        }
    }

    @Override
    public void execute() {
        if (!RobotState.isTest()) {
            double extensionPercent = (arm.getArmState().getExtension() - ArmConstants.ARM_BASE_LENGTH) / (ArmConstants.ARM_LENGTH_EXT - ArmConstants.ARM_BASE_LENGTH);
            double pivotInput = pivotScale * (-controller.getLeftY()) * (1 - 0.75 * extensionPercent);
            if (!startedManualPivot && Math.abs(pivotInput) >= deadzone) startedManualPivot = true;
            if (startedManualPivot) {
                if (Math.abs(pivotInput) <= deadzone && !isHoldingRotation) {
                    isHoldingRotation = true;
                    arm.holdPivot();
                } else if (Math.abs(pivotInput) >= deadzone) {
                    isHoldingRotation = false;
                    arm.setRotatingSpeed(Math.pow(pivotInput, 3) * (1 + ((-1 / ArmConstants.TELESCOPE_FORWARD_LIMIT) * pivotInput))); // TODO: 2/11/2023 Check angles here
                }
            }


            double extensionInput =
                    Math.abs(controller.getLeftTriggerAxis() - controller.getRightTriggerAxis()) < deadzone ? 0
                            : controller.getLeftTriggerAxis() - controller.getRightTriggerAxis();
            double limitedExtensionInput = extensionScale * extensionRateLimiter.calculate(extensionInput);
            if (!startedManualExtension && Math.abs(limitedExtensionInput) >= deadzone) startedManualExtension = true;
            if (startedManualExtension) {
                if (Math.abs(limitedExtensionInput) <= deadzone && !isHoldingExtension) {
                    isHoldingExtension = true;
                    arm.setExtension(arm.getArmState().getExtension());
                } else if (Math.abs(limitedExtensionInput) >= deadzone) {
                    if (!(arm.getMaxExtension() < arm.getArmState().getExtension() && limitedExtensionInput > 0)) {
                        isHoldingExtension = false;
                        arm.setExtendingSpeed(limitedExtensionInput);
                    } else arm.holdExtension();
                }
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopAll();
    }
}