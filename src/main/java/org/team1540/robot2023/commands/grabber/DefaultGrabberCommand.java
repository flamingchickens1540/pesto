package org.team1540.robot2023.commands.grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DefaultGrabberCommand extends CommandBase {
    private final WheeledGrabber wheeledGrabber;

    public DefaultGrabberCommand(WheeledGrabber wheeledGrabber) {
        this.wheeledGrabber = wheeledGrabber;
        addRequirements(wheeledGrabber);
    }

    @Override
    public void initialize() {
        wheeledGrabber.setSpeed(0.1);
        wheeledGrabber.setCurrentLimit(10);
    }

}
