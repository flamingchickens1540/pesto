package org.team1540.robot2023.commands.grabber;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeCommand extends CommandBase {
    private final Intake intake;
    private final XboxController controller;
    private boolean isRunning;

    public IntakeCommand(Intake intake, XboxController controller) {
        this.intake = intake;
        this.controller = controller;
        isRunning = false;
    }

    public void execute() {
        if (controller.getAButtonPressed()) {
            if (isRunning) intake.stop();
            else intake.runIntake();
            isRunning = !isRunning;
        }
    }

    public void end() {
        intake.stop();
    }
}
