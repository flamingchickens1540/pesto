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
//        wheeledGrabber.setCurrentLimit(40);
        wheeledGrabber.setSpeed(1);
    }

    @Override
    public boolean isFinished() {
        return wheeledGrabber.hasGamePiece();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("NO LONGER GRABBY");
    }
}
