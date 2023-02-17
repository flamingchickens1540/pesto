package org.team1540.robot2023;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team1540.robot2023.commands.arm.Arm;
import org.team1540.robot2023.commands.arm.ManualArm;
import org.team1540.robot2023.commands.drivetrain.*;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.commands.drivetrain.PathPlannerDriveCommand;
import org.team1540.robot2023.commands.drivetrain.SwerveDriveCommand;
import org.team1540.robot2023.utils.ButtonPanel;

public class RobotContainer {
    // Hardware

    // Subsystems
    
    Drivetrain drivetrain = new Drivetrain();
    Arm arm = new Arm();
    // Controllers
    CommandXboxController driver = new CommandXboxController(0);
    CommandXboxController copilot = new CommandXboxController(1);
    ButtonPanel controlPanel = new ButtonPanel(2);
    // Commands

    public RobotContainer() {
        initSmartDashboard();
        configureButtonBindings();
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    private void configureButtonBindings() {
        // Driver

        driver.b().onTrue(new InstantCommand(drivetrain::resetAllToAbsolute));
//        new Trigger(driver::getLeftBumper).whileActiveOnce(new IntakeCommand(intake)); //coop:button(LBumper,[HOLD] Intake,pilot)
        driver.a().onTrue(new InstantCommand(drivetrain::zeroGyroscope));
        // Copilot

        controlPanel.onButton(ButtonPanel.PanelButton.TOP_LEFT).whileTrue(new ProxiedGridDriveCommand(drivetrain, 6));
        controlPanel.onButton(ButtonPanel.PanelButton.TOP_RIGHT).whileTrue(new ProxiedGridDriveCommand(drivetrain, 4));

        // SmartDashboard Commands
        
    }

    public void setTeleopDefaultCommands() {
        drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, driver));
        arm.setDefaultCommand(new ManualArm(arm, copilot));
    }

    private void initSmartDashboard() {
        
    }

    public CommandBase getAutonomousCommand() {
        return new PathPlannerDriveCommand(drivetrain);
    }
}
