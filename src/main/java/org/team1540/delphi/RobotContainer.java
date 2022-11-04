package org.team1540.delphi;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {
    // Hardware

    // Subsystems

    // Controllers

    // Commands

    public RobotContainer() {
        initSmartDashboard();
        configureButtonBindings();
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    private void configureButtonBindings() {
        // Driver


        // Copilot


        // Robot hardware button


        // SmartDashboard Commands
        
    }


    private void initSmartDashboard() {
        
    }

    public CommandBase getAutonomousCommand() {
        return new InstantCommand();
    }
}
