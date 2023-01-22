package org.team1540.robot2023.commands.drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static org.team1540.robot2023.utils.MathUtils.deadzone;

public class SwerveDriveCommand extends CommandBase {
    private final Drivetrain drivetrain;
    private final CommandXboxController controller;
    private final double deadzone = 0.1;

    // The rate limit should be relative to the input percent. A value of 1 will take 1 second to get from 0% to 100%, a value of 2 will do that in half a second
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

    
    public SwerveDriveCommand(Drivetrain drivetrain, CommandXboxController controller) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        addRequirements(drivetrain);

    }

    @Override
    public void initialize() {
        xLimiter.reset(0);
        yLimiter.reset(0);
        rotLimiter.reset(0);
    }

    @Override
    public void execute() {
        drivetrain.drive(
                xLimiter.calculate(deadzone(-controller.getLeftY(), deadzone))/2,
                yLimiter.calculate(deadzone(-controller.getLeftX(), deadzone))/2,
                rotLimiter.calculate(-deadzone(controller.getRightX(), deadzone)),
                true
        );
    }

    @Override
    public void end(boolean isInterrupted) {
        drivetrain.drive(0, 0, 0, false);
    }
}