package org.team1540.robot2023;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.commands.drivetrain.PathPlannerDriveCommand;
import org.team1540.robot2023.commands.drivetrain.SwerveDriveCommand;

public class RobotContainer {
    // Hardware

    // Subsystems
    
    Drivetrain drivetrain = new Drivetrain();
//    Elevator elevator = new Elevator();
//    Intake intake = new Intake();
    // Controllers
    XboxController driver = new XboxController(0);
    // Commands

    public RobotContainer() {
        initSmartDashboard();
        configureButtonBindings();
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    private void configureButtonBindings() {
        // Driver


//        new Trigger(driver::getLeftBumper).whileActiveOnce(new IntakeCommand(intake)); //coop:button(LBumper,[HOLD] Intake,pilot)
        new Trigger(driver::getAButton).whenActive(drivetrain::zeroGyroscope);
        // Copilot

        // SmartDashboard Commands
        
    }

    public void setTeleopDefaultCommands() {
//        elevator.setDefaultCommand(new ElevatorMoveCommand(driver, elevator)); //coop:button(LTrigger,[HOLD] Elevator Down,pilot) coop:button(RTrigger,[HOLD] Elevator Up,pilot)
        drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, driver));
    }

    private void initSmartDashboard() {
        
    }

    public CommandBase getAutonomousCommand() {
        return new PathPlannerDriveCommand(drivetrain);
    }
}
