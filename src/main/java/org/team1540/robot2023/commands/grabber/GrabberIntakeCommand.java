package org.team1540.robot2023.commands.grabber;

import org.team1540.lib.RevBlinkin.ColorPattern;
import org.team1540.robot2023.utils.BlinkinManager;
import org.team1540.robot2023.utils.BlinkinManager.ColorPair;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

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
    public boolean isFinished() {
        return wheeledGrabber.hasGamePiece();
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            Commands.sequence(
                BlinkinManager.getInstance().commandSet(ColorPattern.STROBE_BLUE),
                new WaitCommand(1),
                BlinkinManager.getInstance().commandSet(ColorPair.TELEOP)
            ).schedule();
        }
    }
}
