package org.team1540.robot2023;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.commands.drivetrain.PathPlannerDriveCommand;
import org.team1540.robot2023.commands.drivetrain.ProxiedGridDriveCommand;
import org.team1540.robot2023.commands.drivetrain.SwerveDriveCommand;
import org.team1540.robot2023.utils.ButtonPanel;
import org.team1540.robot2023.utils.PolePosition;

public class RobotContainer {
    // Hardware

    // Subsystems

    Drivetrain drivetrain = new Drivetrain();

    // Controllers
    CommandXboxController driver = new CommandXboxController(0);
    ButtonPanel controlPanel = new ButtonPanel(1);
    // Commands

    public RobotContainer() {
        initSmartDashboard();
        configureButtonBindings();
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    private void configureButtonBindings() {
        // Driver
        driver.a().onTrue(new InstantCommand(drivetrain::zeroGyroscope).andThen(drivetrain::resetAllToAbsolute));
        // Copilot

        controlPanel.onButton(ButtonPanel.PanelButton.TOP_LEFT     ).whileTrue(new ProxiedGridDriveCommand(drivetrain, 6, PolePosition.LEFT));
        controlPanel.onButton(ButtonPanel.PanelButton.TOP_CENTER   ).whileTrue(new ProxiedGridDriveCommand(drivetrain, 6, PolePosition.CENTER));
        controlPanel.onButton(ButtonPanel.PanelButton.TOP_RIGHT    ).whileTrue(new ProxiedGridDriveCommand(drivetrain, 6, PolePosition.RIGHT));
        controlPanel.onButton(ButtonPanel.PanelButton.MIDDLE_LEFT  ).whileTrue(new ProxiedGridDriveCommand(drivetrain, PolePosition.LEFT));
        controlPanel.onButton(ButtonPanel.PanelButton.MIDDLE_CENTER).whileTrue(new ProxiedGridDriveCommand(drivetrain, PolePosition.CENTER));
        controlPanel.onButton(ButtonPanel.PanelButton.MIDDLE_RIGHT ).whileTrue(new ProxiedGridDriveCommand(drivetrain, PolePosition.RIGHT));


        // SmartDashboard Commands

    }

    public void setTeleopDefaultCommands() {
        drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, driver));
    }

    private void initSmartDashboard() {

    }

    public CommandBase getAutonomousCommand() {
        return new PathPlannerDriveCommand(drivetrain);
    }
}
