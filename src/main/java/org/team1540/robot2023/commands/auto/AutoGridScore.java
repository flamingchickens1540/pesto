package org.team1540.robot2023.commands.auto;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team1540.robot2023.commands.arm.*;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.commands.grabber.DefaultGrabberCommand;
import org.team1540.robot2023.commands.grabber.GrabberOuttakeCommand;
import org.team1540.robot2023.commands.grabber.WheeledGrabber;
import org.team1540.robot2023.utils.ArmState;
import org.team1540.robot2023.utils.GridScoreData;
import org.team1540.robot2023.utils.PolePosition;

import java.util.Objects;

public class AutoGridScore extends SequentialCommandGroup {

    public AutoGridScore(Drivetrain drivetrain, Arm arm, GridScoreData positions, WheeledGrabber intake) {
        this(drivetrain, arm, positions, intake, null, true);
    }
    public AutoGridScore(Drivetrain drivetrain, Arm arm, GridScoreData positions, WheeledGrabber intake, CommandXboxController controller) {
        this(drivetrain, arm, positions, intake, controller, true);
    }
    public AutoGridScore(Drivetrain drivetrain, Arm arm, GridScoreData positions, WheeledGrabber intake, CommandXboxController controller, boolean shouldAlign){

        addCommands(
            Commands.race(
                    new DefaultGrabberCommand(intake),
                    Commands.sequence(
                            new ProxyCommand(() -> {
                                Translation2d endPoint = AutoDrive.getGridDrivePose(drivetrain, positions);
                                return AutoDrive.driveToPoints(
                                        drivetrain,
//                                        new PathPoint(endPoint.plus(new Translation2d(0.127,0)), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)),
                                        new PathPoint(endPoint, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180))
                                );
                            }
                            ).unless(()->!shouldAlign),
//                            new ProxiedGridDriveCommand(drivetrain, positions),
                            new RetractAndPivotCommand(arm, positions.approach.getRotation2d()),
                            new ExtensionCommand(arm, positions.approach),
                            new WaitUntilCommand(() -> controller.getLeftTriggerAxis() > 0.95).unless(() -> controller == null || positions.polePosition == PolePosition.CENTER),
                            new PivotCommand(arm,catchNull(positions.score)).unless(() -> positions.score == null)
                    )
            ),
            Commands.race(
                new GrabberOuttakeCommand(intake),
                Commands.sequence(
                        new WaitCommand(0.25),

                    new PivotCommand(arm, catchNull(positions.retreat)).unless(() -> positions.retreat == null),
                    new ResetArmPositionCommand(arm)
//                    new ResetArmPositionCommand(arm)
                )
            )
        );
    }

    private ArmState catchNull(ArmState state) {
        return Objects.requireNonNullElseGet(state, () -> ArmState.fromRotationExtension(new Rotation2d(), 0));
    }
}
