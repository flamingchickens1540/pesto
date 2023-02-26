package org.team1540.robot2023.commands.arm;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team1540.robot2023.Constants.ArmConstants;

public class ManualExtension extends CommandBase {
    private final SlewRateLimiter slewRateLimiter = new SlewRateLimiter(1,-1.5,0);
    private final Telescope telescope;
    private final CommandXboxController controller;
    private boolean isHolding;

    public ManualExtension(Telescope telescope, CommandXboxController controller){
        this.telescope = telescope;
        this.controller = controller;
        addRequirements(telescope);
    }

    @Override
    public void initialize() {
        isHolding = false;
    }

    @Override
    public void execute() {
        if(controller.getLeftTriggerAxis() == 0 && controller.getRightTriggerAxis() == 0 && !isHolding){
            isHolding = true;
            telescope.setExtension(telescope.getExtension());
        }
        else {
            isHolding = false;
            telescope.setExtendingSpeed(slewRateLimiter.calculate(controller.getLeftTriggerAxis() - controller.getRightTriggerAxis()));
        }
    }

    @Override
    public void end(boolean interrupted) {
        telescope.stop();
    }
}