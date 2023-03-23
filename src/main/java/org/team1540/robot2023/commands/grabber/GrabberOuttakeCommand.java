package org.team1540.robot2023.commands.grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class GrabberOuttakeCommand extends CommandBase {
    private final WheeledGrabber wheeledGrabber;
    private final double speed;

    public GrabberOuttakeCommand(WheeledGrabber wheeledGrabber){
        this(wheeledGrabber, 0.3);
    }

    public GrabberOuttakeCommand(WheeledGrabber wheeledGrabber, double speed){
        this.speed = speed;
        this.wheeledGrabber = wheeledGrabber;
        addRequirements(wheeledGrabber);
    }

    @Override
    public void initialize() {
        wheeledGrabber.setSpeed(-speed);
    }

    @Override
    public void end(boolean interrupted) {
    }
}
