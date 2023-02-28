package org.team1540.robot2023.commands.grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class GrabberIntakeCommand extends CommandBase {
    private final WheeledGrabber wheeledGrabber;

    public GrabberIntakeCommand(WheeledGrabber wheeledGrabber) {
        this.wheeledGrabber = wheeledGrabber;
        addRequirements(wheeledGrabber);
    }

    @Override
    public void initialize() {
        wheeledGrabber.setSpeed(1);
    }

    @Override
    public void execute() {
        if (wheeledGrabber.hasGamePiece()) {
            wheeledGrabber.setSpeed(0.1);
        }
    }

    @Override
    public void end(boolean interrupted) {
        wheeledGrabber.stop();
    }
}
