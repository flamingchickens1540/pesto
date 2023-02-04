package org.team1540.robot2023.commands.grabber;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClawCommand extends CommandBase {
    private final Claw claw;
    private final XboxController controller;
    private boolean isOpen;

    public ClawCommand(Claw claw, XboxController controller) {
        this.claw = claw;
        this.controller = controller;
        isOpen = false;
    }

    public void initialize() {
        claw.set(false);
    }

    public void execute() {
        if (controller.getAButtonPressed()) {
            claw.set(!isOpen);
            isOpen = !isOpen;
        }
    }
}
