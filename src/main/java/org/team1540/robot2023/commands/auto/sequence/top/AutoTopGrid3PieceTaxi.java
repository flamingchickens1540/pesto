package org.team1540.robot2023.commands.auto.sequence.top;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import org.team1540.robot2023.Constants;
import org.team1540.robot2023.commands.arm.Arm;
import org.team1540.robot2023.commands.arm.PivotCommand;
import org.team1540.robot2023.commands.arm.ResetArmPositionCommand;
import org.team1540.robot2023.commands.arm.SetArmPosition;
import org.team1540.robot2023.commands.auto.AutoCube;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.commands.grabber.GrabberIntakeCommand;
import org.team1540.robot2023.commands.grabber.GrabberOuttakeCommand;
import org.team1540.robot2023.commands.grabber.WheeledGrabber;
import org.team1540.robot2023.utils.ArmState;
import org.team1540.robot2023.utils.AutoCommand;
import org.team1540.robot2023.utils.PolePosition;

import java.util.List;

public class AutoTopGrid3PieceTaxi extends AutoCommand {
    public AutoTopGrid3PieceTaxi(Drivetrain drivetrain, Arm arm, WheeledGrabber intake) {
        List<Command> pathCommands = getPathPlannerDriveCommandGroup(drivetrain, "TopGrid3PieceTaxi", new PathConstraints(5, 3), false);
        addCommands(
                new AutoCube(drivetrain, arm, Constants.Auto.highCube.withPolePosition(PolePosition.CENTER), intake, false),
                Commands.parallel(
                        new GrabberIntakeCommand(intake),
                        Commands.sequence(
                                Commands.parallel(
                                    pathCommands.get(0),
                                    Commands.sequence(
                                            new ResetArmPositionCommand(arm),
                                            new PivotCommand(arm, Constants.Auto.armDownBackwards)
                                    )
                                ),
                                Commands.parallel(
                                    new SetArmPosition(arm, Constants.Auto.midCube.approach),
                                    pathCommands.get(1)
                                )
                        )
                ),
                new InstantCommand(drivetrain::updateWithScoringApriltags),//                new AutoCube(drivetrain, arm, Constants.Auto.midCube.withPolePosition(PolePosition.CENTER), intake, false),
                Commands.parallel(
                        pathCommands.get(2),
                        Commands.sequence(
                                new GrabberOuttakeCommand(intake, 1).withTimeout(0.1),
                                new PrintCommand("This is happeningf"),
                                new ResetArmPositionCommand(arm),
                                new SetArmPosition(arm, Constants.Auto.reverseCube),
                                new GrabberIntakeCommand(intake)
                        )
                ),
                Commands.parallel(
                                new SetArmPosition(arm, new ArmState(0, Rotation2d.fromDegrees(-90))),
                        pathCommands.get(3)
                ),
                new InstantCommand(drivetrain::updateWithScoringApriltags),
                new GrabberOuttakeCommand(intake,0.5)
//                new AutoHybrid(drivetrain, arm, Constants.Auto.hybridNode.withPolePosition(PolePosition.CENTER), intake, null, true)

        );
    }


}