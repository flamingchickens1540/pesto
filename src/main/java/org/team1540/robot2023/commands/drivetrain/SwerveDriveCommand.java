package org.team1540.robot2023.commands.drivetrain;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveCommand extends CommandBase {
    private final Drivetrain drivetrain;
    private final XboxController controller;
    
    public SwerveDriveCommand(Drivetrain drivetrain, XboxController controller) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.drive(-controller.getLeftY()/2, -controller.getLeftX()/2, -Math.toRadians(controller.getRightX()*100), true);
    }

    @Override
    public void end(boolean isInterrupted) {
        drivetrain.drive(0, 0, 0, false);
    }
}