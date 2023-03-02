package org.team1540.robot2023.commands.arm;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import org.team1540.robot2023.Constants;
import org.team1540.robot2023.Constants.ArmConstants;

public class ManualArm extends CommandBase {
    private final SlewRateLimiter slewRateLimiter = new SlewRateLimiter(1,-1,0);
    private final Arm arm;
    private final CommandXboxController controller;
    private final double deadzone = 0.1;
    private boolean isHoldingRotation;
    private boolean isHoldingExtension;

    private SlewRateLimiter rotationOutputRateLimiter = new SlewRateLimiter(Constants.ArmConstants.PIVOT_FORWARD_LIMIT,Constants.ArmConstants.PIVOT_REVERSE_LIMIT,0);

    public ManualArm(Arm arm, CommandXboxController controller){
        this.arm = arm;
        this.controller = controller;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        isHoldingRotation = false;
        isHoldingExtension = false;
    }

    @Override
    public void execute() {
        double pivotInput = -controller.getLeftY();
        if (Math.abs(pivotInput) <= deadzone && !isHoldingRotation) {
            isHoldingRotation = true;
            arm.setRotation(arm.getArmState().getRotation2d());
        } else if (Math.abs(pivotInput) >= deadzone) {
            isHoldingRotation = false;
            // 0 = fully extended. 1 = fully retracted
            double percentExtendedInverted = 1-(arm.getArmState().getExtension() - ArmConstants.ARM_BASE_LENGTH) / (ArmConstants.ARM_LENGTH_EXT - ArmConstants.ARM_BASE_LENGTH);
            double adjustedPivotInput = Constants.ArmConstants.PIVOT_EXT_MAX_SPEED + percentExtendedInverted * (Constants.ArmConstants.PIVOT_BASE_MAX_SPEED - Constants.ArmConstants.PIVOT_EXT_MAX_SPEED);
            System.out.println("NON-r-limited pivot speed: " + adjustedPivotInput);
            double rotatingSpeed = rotationOutputRateLimiter.calculate(adjustedPivotInput);
            System.out.println("rate-limited pivot speed: " + rotatingSpeed);
            arm.setRotatingSpeed(rotatingSpeed); // TODO: 2/11/2023 Check angles here
        }

        double limitedExtensionInput = slewRateLimiter.calculate(controller.getLeftTriggerAxis() - controller.getRightTriggerAxis());
        if(Math.abs(limitedExtensionInput) <= deadzone && !isHoldingExtension){
            isHoldingExtension = true;
            arm.setExtension(arm.getArmState().getExtension());
        }
        else if (Math.abs(limitedExtensionInput) >= deadzone){
            isHoldingExtension = false;
            arm.setExtendingSpeed(limitedExtensionInput);
        }
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopAll();
    }
}