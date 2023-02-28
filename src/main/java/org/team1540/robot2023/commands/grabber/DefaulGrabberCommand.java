package org.team1540.robot2023.commands.grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DefaulGrabberCommand extends CommandBase {
    private final WheeledGrabber wheeledGrabber;

    public DefaulGrabberCommand(WheeledGrabber wheeledGrabber) {
        this.wheeledGrabber = wheeledGrabber;
        addRequirements(wheeledGrabber);
    }

    @Override
    public void initialize() {
        wheeledGrabber.setSpeed(wheeledGrabber.hasGamePiece() ? 0.1 : 0);
    }

    @Override
    public void end(boolean interrupted) {
        wheeledGrabber.stop();
    }
}
