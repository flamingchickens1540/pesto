package org.team1540.robot2023.commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import org.team1540.robot2023.commands.arm.*;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.commands.drivetrain.ProxiedGridDriveCommand;
import org.team1540.robot2023.commands.grabber.DefaultGrabberCommand;
import org.team1540.robot2023.commands.grabber.GrabberOuttakeCommand;
import org.team1540.robot2023.commands.grabber.WheeledGrabber;
import org.team1540.robot2023.utils.ArmState;
import org.team1540.robot2023.utils.GridScoreData;
import org.team1540.robot2023.utils.PolePosition;

import java.util.Objects;

public class AutoGridScore extends SequentialCommandGroup {

    public AutoGridScore(Drivetrain drivetrain, Arm arm, PolePosition polePosition, GridScoreData positions, WheeledGrabber intake) {
        this(drivetrain, arm, polePosition, positions, intake, true);
    }
    public AutoGridScore(Drivetrain drivetrain, Arm arm, PolePosition polePosition, GridScoreData positions, WheeledGrabber intake, boolean shouldAlign){
        Command alignmentCommand = shouldAlign ? new ProxiedGridDriveCommand(drivetrain, polePosition) : new InstantCommand();

        addCommands(
            Commands.race(
                    new DefaultGrabberCommand(intake),
                    Commands.sequence(
                            alignmentCommand,
                            new RetractAndPivotCommand(arm, positions.approach.getRotation2d()),
                            new ExtensionCommand(arm, positions.approach),
                            new PivotCommand(arm,catchNull(positions.score)).unless(() -> positions.score == null)
                    )
            ),
            Commands.race(
                new GrabberOuttakeCommand(intake),
                Commands.sequence(
                        new WaitCommand(0.25),
                    new PivotCommand(arm, catchNull(positions.retreat)).unless(() -> positions.retreat == null),
                    new ResetArmPositionCommand(arm)
                )
            )
        );
    }

    private ArmState catchNull(ArmState state) {
        return Objects.requireNonNullElseGet(state, () -> ArmState.fromRotationExtension(new Rotation2d(), 0));
    }
}
