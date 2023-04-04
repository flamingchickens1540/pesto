package org.team1540.robot2023.commands.auto;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team1540.robot2023.commands.arm.Arm;
import org.team1540.robot2023.commands.arm.ResetArmPositionCommand;
import org.team1540.robot2023.commands.arm.SetArmPosition;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.commands.grabber.GrabberAggressiveCommand;
import org.team1540.robot2023.commands.grabber.GrabberOuttakeCommand;
import org.team1540.robot2023.commands.grabber.WheeledGrabber;
import org.team1540.robot2023.utils.GridScoreData;

import java.util.function.BooleanSupplier;

public class AutoCone extends SequentialCommandGroup {
    public AutoCone(Drivetrain drivetrain, Arm arm, GridScoreData positions, WheeledGrabber intake, CommandXboxController controller, boolean shouldAlign){
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

                                Commands.deadline(
                                        new SetArmPosition(arm, positions.approach),
                                        new GrabberAggressiveCommand(intake)
                                ),
                                new WaitUntilCommand(() -> controller.getLeftTriggerAxis() > 0.95).unless(() -> controller == null),
                                new SetArmPosition(arm, AutoHybrid.catchNull(positions.score)).unless(() -> positions.score == null),
                                Commands.deadline(
                                        Commands.sequence(
                                                new WaitCommand(0.25),
                                                new SetArmPosition(arm, AutoHybrid.catchNull(positions.retreat)).unless(() -> positions.retreat == null),
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
}
