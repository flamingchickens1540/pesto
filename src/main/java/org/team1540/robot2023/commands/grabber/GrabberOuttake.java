package org.team1540.robot2023.commands.grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class GrabberOuttake extends CommandBase {
    private final WheeledGrabber wheeledGrabber;

    public GrabberOuttake(WheeledGrabber wheeledGrabber){
        this.wheeledGrabber = wheeledGrabber;
        addRequirements(wheeledGrabber);
    }

    @Override
    public void initialize() {
        wheeledGrabber.setSpeed(-0.7);
    }

    @Override
    public void end(boolean interrupted) {
        wheeledGrabber.setSpeed(0);
    }
}
