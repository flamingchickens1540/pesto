package org.team1540.delphi.commands.drivetrain;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveCommand extends CommandBase {
    private final Drivetrain drivetrain;
    private final XboxController controller;
    
    public SwerveDriveCommand(Drivetrain drivetrain, XboxController controller) {
        this.drivetrain = drivetrain;
        this.controller = controller;
    }

    @Override
    public void execute() {
        drivetrain.drive(controller.getLeftX(), controller.getLeftY(), controller.getRightX(), false);
    }

    @Override
    public void end(boolean isInterrupted) {
        
    }
}