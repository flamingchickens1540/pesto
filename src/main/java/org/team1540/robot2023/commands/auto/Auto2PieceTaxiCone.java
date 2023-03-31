package org.team1540.robot2023.commands.auto;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.team1540.robot2023.Constants;
import org.team1540.robot2023.commands.arm.Arm;
import org.team1540.robot2023.commands.arm.PivotCommand;
import org.team1540.robot2023.commands.arm.ResetArmPositionCommand;
import org.team1540.robot2023.commands.arm.RetractAndPivotCommand;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.commands.grabber.GrabberIntakeCommand;
import org.team1540.robot2023.commands.grabber.WheeledGrabber;
import org.team1540.robot2023.utils.AutoCommand;
import org.team1540.robot2023.utils.PolePosition;
import org.team1540.robot2023.utils.ScoringGridLocation;

import java.util.List;

public class Auto2PieceTaxiCone extends AutoCommand {
    public Auto2PieceTaxiCone(Drivetrain drivetrain, Arm arm, WheeledGrabber intake, ScoringGridLocation.OuterGrid grid) {
        List<Command> pathCommands = getPathPlannerDriveCommandGroup(drivetrain, grid.getPathName("2PieceTaxiCone"), new PathConstraints(4, 2), true);
        addCommands(

                new AutoCone(drivetrain, arm, Constants.Auto.highCone.withPolePosition(PolePosition.LEFT), intake, null, false),
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
                                        new RetractAndPivotCommand(arm, Constants.Auto.highCube.approach),
                                    pathCommands.get(1)
                                )
                        )
                ),
                new AutoCube(drivetrain, arm, Constants.Auto.highCube.withPolePosition(PolePosition.CENTER), intake, true)

        );
    }


}