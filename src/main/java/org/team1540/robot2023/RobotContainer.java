package org.team1540.robot2023;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team1540.robot2023.commands.drivetrain.CardinalSwerveDriveCommand;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.commands.drivetrain.PathPlannerDriveCommand;
import org.team1540.robot2023.commands.drivetrain.SwerveDriveCommand;
import org.team1540.robot2023.utils.ButtonPanel;

public class RobotContainer {
    // Hardware

    // Subsystems
    
    Drivetrain drivetrain = new Drivetrain();
//    Elevator elevator = new Elevator();
//    Intake intake = new Intake();
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

        driver.b().onTrue(new InstantCommand(drivetrain::resetAllToAbsolute));
//        new Trigger(driver::getLeftBumper).whileActiveOnce(new IntakeCommand(intake)); //coop:button(LBumper,[HOLD] Intake,pilot)
        driver.a().onTrue(new InstantCommand(drivetrain::zeroGyroscope));
        // Copilot

        controlPanel.onButton(ButtonPanel.PanelButton.TOP_LEFT).whileTrue(new PrintCommand("1")).onFalse(new PrintCommand("-1"));
        controlPanel.onButton(2).whileTrue(new PrintCommand("2")).onFalse(new PrintCommand("-2"));
        controlPanel.onButton(3).whileTrue(new PrintCommand("3")).onFalse(new PrintCommand("-3"));
        controlPanel.onButton(4).whileTrue(new PrintCommand("4")).onFalse(new PrintCommand("-4"));
        controlPanel.onAnyGrid().whileTrue(new PrintCommand("ANY")).onFalse(new PrintCommand("-ANY"));
        // SmartDashboard Commands
        
    }

    public void setTeleopDefaultCommands() {
//        elevator.setDefaultCommand(new ElevatorMoveCommand(driver, elevator)); //coop:button(LTrigger,[HOLD] Elevator Down,pilot) coop:button(RTrigger,[HOLD] Elevator Up,pilot)
//        drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, driver));
        drivetrain.setDefaultCommand(new CardinalSwerveDriveCommand(drivetrain, driver));
    }

    private void initSmartDashboard() {
        
    }

    public CommandBase getAutonomousCommand() {
        return new PathPlannerDriveCommand(drivetrain);
    }
}
