package org.team1540.robot2023.commands.grabber;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DefaultGrabberCommand extends SequentialCommandGroup {
    private final WheeledGrabber wheeledGrabber;

    public DefaultGrabberCommand(WheeledGrabber wheeledGrabber) {
        this.wheeledGrabber = wheeledGrabber;
        addRequirements(wheeledGrabber);
        addCommands(
                new InstantCommand(() -> {
                    wheeledGrabber.setSpeed(0.5);
                    wheeledGrabber.setCurrentLimit(20);
                }),
                Commands.repeatingSequence(
                        new WaitCommand(0.3),
                        new InstantCommand(() -> wheeledGrabber.setCurrentLimit(20)),
                        new WaitCommand(0.7),
                        new InstantCommand(() -> wheeledGrabber.setCurrentLimit(30))
                )
        );
    }


}
