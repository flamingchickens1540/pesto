package org.team1540.delphi.commands.elevator;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ElevatorMoveCommand extends CommandBase {
    private final XboxController controller;
    private final Elevator elevator;

    public ElevatorMoveCommand(XboxController controller, Elevator elevator) {
        this.controller = controller;
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        // TODO: Keep elevator stable with no input
        elevator.setPercent(controller.getLeftTriggerAxis()-controller.getRightTriggerAxis());
    }

    @Override
    public void end(boolean isInterrupted) {
        elevator.setPercent(0);
    }
}
