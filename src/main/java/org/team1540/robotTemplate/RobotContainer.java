package org.team1540.robotTemplate;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    // Hardware

    // Subsystems

    // Controllers

    // Commands

    public RobotContainer() {
        initSmartDashboard();
        configureButtonBindings();
        initModeTransitionBindings();
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    private void configureButtonBindings() {
        // Driver


        // Copilot


        // Robot hardware button


        // SmartDashboard Commands
        
    }

    private void initModeTransitionBindings() {
        Trigger enabled = new Trigger(RobotState::isEnabled);
        Trigger disabled = new Trigger(DriverStation::isDisabled);
        Trigger fmsConnected = new Trigger(DriverStation::isFMSAttached);

        Trigger autonomous = new Trigger(DriverStation::isAutonomousEnabled);
        Trigger teleop = new Trigger(DriverStation::isTeleopEnabled);
        Trigger endgame = new Trigger(() -> Timer.getMatchTime() <= 30).and(teleop);
    }

    private void initSmartDashboard() {
        
    }

    public CommandBase getAutonomousCommand() {
        return new InstantCommand();
    }
}
