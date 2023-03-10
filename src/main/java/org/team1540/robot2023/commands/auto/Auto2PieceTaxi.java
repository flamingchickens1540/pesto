package org.team1540.robot2023.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.team1540.robot2023.Constants;
import org.team1540.robot2023.commands.arm.Arm;
import org.team1540.robot2023.commands.arm.SetArmPosition;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.commands.grabber.GrabberIntakeCommand;
import org.team1540.robot2023.commands.grabber.WheeledGrabber;
import org.team1540.robot2023.utils.AutoCommand;
import org.team1540.robot2023.utils.PolePosition;
import org.team1540.robot2023.utils.ScoringGridLocation;

import java.util.List;

public class Auto2PieceTaxi extends AutoCommand {
    public Auto2PieceTaxi(Drivetrain drivetrain, Arm arm, WheeledGrabber intake, ScoringGridLocation.OuterGrid grid) {
        List<Command> pathCommands = getPathPlannerDriveCommandGroup(drivetrain, grid.getPathName("2PieceTaxi"));
        addCommands(

                new AutoGridScore(drivetrain, arm, Constants.Auto.highCube.withPolePosition(PolePosition.CENTER), intake),
                new InstantCommand(drivetrain::stopTags),
                Commands.parallel(
                        new GrabberIntakeCommand(intake),

                        Commands.sequence(
                                Commands.parallel(
                            pathCommands.get(0),
                                    new SetArmPosition(arm, Constants.Auto.armUp)
                                ),
                                new SetArmPosition(arm, Constants.Auto.armDownBackwards),
                                pathCommands.get(1),
                                new SetArmPosition(arm, Constants.Auto.armUp),
                                pathCommands.get(2)
                        )
                ),
                new InstantCommand(drivetrain::startTags),
                new AutoGridScore(drivetrain, arm, Constants.Auto.midCube.withPolePosition(PolePosition.CENTER), intake)

        );
    }


}