package org.team1540.robot2023.commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team1540.robot2023.commands.arm.Arm;
import org.team1540.robot2023.commands.arm.ResetArmPositionCommand;
import org.team1540.robot2023.commands.arm.SetArmPosition;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.commands.grabber.GrabberOuttakeCommand;
import org.team1540.robot2023.commands.grabber.WheeledGrabber;
import org.team1540.robot2023.utils.ArmState;
import org.team1540.robot2023.utils.GridScoreData;

import java.util.Objects;
import java.util.function.BooleanSupplier;

public class AutoHybrid extends SequentialCommandGroup {

    public AutoHybrid(Drivetrain drivetrain, Arm arm, GridScoreData positions, WheeledGrabber intake) {
        this(drivetrain, arm, positions, intake, null, true);
    }
    public AutoHybrid(Drivetrain drivetrain, Arm arm, GridScoreData positions, WheeledGrabber intake, CommandXboxController controller) {
        this(drivetrain, arm, positions, intake, controller, true);
    }
    public AutoHybrid(Drivetrain drivetrain, Arm arm, GridScoreData positions, WheeledGrabber intake, CommandXboxController controller, boolean shouldAlign){
        BooleanSupplier shouldRun;
        if (shouldAlign) {
            shouldRun = drivetrain::updateWithScoringApriltags;
        } else {
            shouldRun = () -> true;
        }
        addCommands(
            new ConditionalCommand(
                    Commands.sequence(
                            new AutoGridAlign(drivetrain, positions, shouldAlign),
        //                            new ProxiedGridDriveCommand(drivetrain, positions),
                            new SetArmPosition(arm, positions.approach),
                            new SetArmPosition(arm,catchNull(positions.score)).unless(() -> positions.score == null),
                            Commands.deadline(
                                    Commands.sequence(
                                            new WaitCommand(0.25),
                                            new SetArmPosition(arm, catchNull(positions.retreat)).unless(() -> positions.retreat == null),
                                            new ResetArmPositionCommand(arm, false),
                                            new ResetArmPositionCommand(arm, true)
                                    ),
                                    new GrabberOuttakeCommand(intake)
                            )
            ),
            new InstantCommand(),
            shouldRun
            )
        );
    }

    public static ArmState catchNull(ArmState state) {
        return Objects.requireNonNullElseGet(state, () -> ArmState.fromRotationExtension(new Rotation2d(), 0));
    }
}
