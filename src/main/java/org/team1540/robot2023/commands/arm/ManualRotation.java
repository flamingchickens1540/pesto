package org.team1540.robot2023.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team1540.robot2023.Constants.ArmConstants;

public class ManualRotation extends CommandBase {
    private final Arm arm;
    private final CommandXboxController controller;
    private static final double deadzone = 0.1;
    private boolean isHolding;

    public ManualRotation(Arm arm, CommandXboxController controller){
        this.arm = arm;
        this.controller = controller;
        addRequirements(arm.pivot);
    }

    @Override
    public void initialize() {
        isHolding = false;
    }

    @Override
    public void execute() {
        double pivotInput = -controller.getLeftY();
        if (Math.abs(pivotInput) <= deadzone && !isHolding) {
            isHolding = true;
            arm.pivot.setRotation(arm.getArmState().getRotation2d());
        } else if (Math.abs(pivotInput) > deadzone) {
            isHolding = false;
            arm.pivot.setRotatingSpeed(Math.pow(pivotInput, 3) * (1 + ((-1/ArmConstants.TELESCOPE_FORWARD_LIMIT)*pivotInput))); // TODO: 2/11/2023 Check angles here
        }
    }

    @Override
    public void end(boolean interrupted) {
        arm.pivot.stop();
    }
}