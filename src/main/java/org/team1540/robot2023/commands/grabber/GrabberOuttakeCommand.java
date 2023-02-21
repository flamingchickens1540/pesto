package org.team1540.robot2023.commands.grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class GrabberOuttakeCommand extends CommandBase {
    private final WheeledGrabber wheeledGrabber;

    public GrabberOuttakeCommand(WheeledGrabber wheeledGrabber){
        this.wheeledGrabber = wheeledGrabber;
        addRequirements(wheeledGrabber);
    }

    @Override
    public void initialize() {
        wheeledGrabber.setSpeed(-0.3);
    }

    @Override
    public void end(boolean interrupted) {
        wheeledGrabber.setSpeed(0);
    }
}
