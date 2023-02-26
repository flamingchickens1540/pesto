package org.team1540.robot2023.commands.arm;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team1540.robot2023.Constants.ArmConstants;

public class ManualPivot extends CommandBase {
    private final SlewRateLimiter slewRateLimiter = new SlewRateLimiter(1,-1.5,0);
    private final Pivot pivot;
    private final CommandXboxController controller;
    private final double deadzone = 0.1;
    private boolean isHolding;

    public ManualPivot(Pivot pivot, CommandXboxController controller){
        this.pivot = pivot;
        this.controller = controller;
        addRequirements(pivot);
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
            pivot.setRotation(pivot.getRotation2d());
        } else if (Math.abs(pivotInput) > deadzone) {
            isHolding = false;
            pivot.setRotatingSpeed(Math.pow(pivotInput, 3) * (1 + ((-1/ArmConstants.TELESCOPE_FORWARD_LIMIT)*pivotInput))); // TODO: 2/11/2023 Check angles here
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        pivot.stop();
    }
}