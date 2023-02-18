package org.team1540.robot2023;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team1540.robot2023.commands.arm.*;
import org.team1540.robot2023.commands.drivetrain.*;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.commands.drivetrain.PathPlannerDriveCommand;
import org.team1540.robot2023.commands.drivetrain.SwerveDriveCommand;
import org.team1540.robot2023.commands.grabber.GrabberOuttake;
import org.team1540.robot2023.commands.grabber.WheeledGrabber;
import org.team1540.robot2023.commands.grabber.GrabberIntake;
import org.team1540.robot2023.utils.ButtonPanel;

public class RobotContainer {
    // Hardware

    // Subsystems
    
    Drivetrain drivetrain = new Drivetrain();
    Arm arm = new Arm();
    WheeledGrabber wheeledGrabber = new WheeledGrabber();
//    PneumaticClaw pneumaticClaw = new PneumaticClaw();
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


        copilot.a().toggleOnTrue(new GrabberIntake(wheeledGrabber));
        copilot.b().whileTrue(new GrabberOuttake(wheeledGrabber));

        //Pneumatic Control
//        copilot.a().onTrue(new InstantCommand(() -> pneumaticClaw.toggle()));
//        copilot.a().onTrue(new InstantCommand(() -> pneumaticClaw.set(true)));
//        copilot.b().onTrue(new InstantCommand(() -> pneumaticClaw.set(false)));


        copilot.x().whileTrue(new ExtensionPID(arm, 30));
        copilot.y().whileTrue(new ExtensionPID(arm, 40));
        copilot.rightBumper().whileTrue(new PivotPID(arm, Rotation2d.fromDegrees(90)));
        copilot.leftBumper().whileTrue(new PivotPID(arm, Rotation2d.fromDegrees(0)));
        copilot.leftStick().onTrue(new InstantCommand(() -> arm.resetAngle())); // TODO: 2/18/2023 change binding

        // SmartDashboard Commands
        
    }

    public void setTeleopDefaultCommands() {
        drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, driver));
        arm.setDefaultCommand(new ManualArm(arm, copilot));
    }

    private void initSmartDashboard() {
        SmartDashboard.putNumber("arm/targetX", 22);
        SmartDashboard.putNumber("arm/targetY", 0);
        SmartDashboard.putNumber("arm/targetAngle", 0);
        SmartDashboard.putNumber("arm/targetExtension", 22);
    }

    public CommandBase getAutonomousCommand() {
        return new PathPlannerDriveCommand(drivetrain);
    }
}
