package org.team1540.robot2023.commands.drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static org.team1540.robot2023.utils.MathUtils.deadzone;

public class SwerveDriveCommand extends CommandBase {
    private final Drivetrain drivetrain;
    private final XboxController controller;
    private final double deadzone = 0.01;
//    private final SlewRateLimiter xLimiter = new SlewRateLimiter(0.999);
//    private final SlewRateLimiter yLimiter = new SlewRateLimiter(0.999);
//    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(0.999);

    
    public SwerveDriveCommand(Drivetrain drivetrain, XboxController controller) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        addRequirements(drivetrain);

    }

    @Override
    public void initialize() {
//        xLimiter.reset(0);
//        yLimiter.reset(0);
//        rotLimiter.reset(0);
    }

    @Override
    public void execute() {
        drivetrain.drive(
//                xLimiter.calculate(deadzone(-controller.getLeftY(), deadzone))/2,
//                yLimiter.calculate(deadzone(-controller.getLeftX(), deadzone))/2,
//                -Math.toRadians(rotLimiter.calculate(deadzone(controller.getRightX(), deadzone))*360), true);
                deadzone(-controller.getLeftY(), deadzone)/2,
                deadzone(-controller.getLeftX(), deadzone)/2,
                deadzone(controller.getRightX(), deadzone), true);
    }

    @Override
    public void end(boolean isInterrupted) {
        drivetrain.drive(0, 0, 0, false);
    }
}