package org.team1540.robot2023.commands.auto.sequence;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.team1540.robot2023.Constants;
import org.team1540.robot2023.commands.arm.Arm;
import org.team1540.robot2023.commands.arm.PivotCommand;
import org.team1540.robot2023.commands.arm.ResetArmPositionCommand;
import org.team1540.robot2023.commands.arm.RetractAndPivotCommand;
import org.team1540.robot2023.commands.auto.AutoCone;
import org.team1540.robot2023.commands.auto.AutoCube;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.commands.grabber.GrabberIntakeCommand;
import org.team1540.robot2023.commands.grabber.WheeledGrabber;
import org.team1540.robot2023.commands.vision.DriveToGamePiece;
import org.team1540.robot2023.commands.vision.TurnToGamePiece;
import org.team1540.robot2023.utils.*;

import java.util.List;

public class Auto2PieceTaxiConeVision extends AutoCommand {
    public Auto2PieceTaxiConeVision(Drivetrain drivetrain, Arm arm, WheeledGrabber intake, ScoringGridLocation.OuterGrid grid, Limelight limelight) {
        List<Command> pathCommands = getPathPlannerDriveCommandGroup(drivetrain, grid.getPathName("2PieceTaxiConeVision"), new PathConstraints(4, 2), true);
        limelight.setPipeline(Limelight.Pipeline.GAME_PIECE);
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
                                Commands.sequence(
                                        new TurnToGamePiece(drivetrain, null, GamePiece.CUBE ),
                                        new DriveToGamePiece(drivetrain, () -> Constants.Auto.autoDriveDistance)        
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