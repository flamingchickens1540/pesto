package org.team1540.robot2023.commands.auto;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team1540.robot2023.Constants;
import org.team1540.robot2023.commands.arm.Arm;
import org.team1540.robot2023.commands.arm.PivotCommand;
import org.team1540.robot2023.commands.arm.ResetArmPositionCommand;
import org.team1540.robot2023.commands.arm.SetArmPosition;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.commands.grabber.GrabberIntakeCommand;
import org.team1540.robot2023.commands.grabber.GrabberOuttakeCommand;
import org.team1540.robot2023.commands.grabber.WheeledGrabber;
import org.team1540.robot2023.commands.vision.DriveToGamePiece;
import org.team1540.robot2023.commands.vision.TurnToGamePiece;
import org.team1540.robot2023.utils.AutoCommand;
import org.team1540.robot2023.utils.Limelight;
import org.team1540.robot2023.utils.PolePosition;

import java.util.List;

public class AutoBottomGrid2_5PieceTaxiConeVision extends AutoCommand {
    public AutoBottomGrid2_5PieceTaxiConeVision(Drivetrain drivetrain, Arm arm, WheeledGrabber intake, Limelight limelight) {
        List<Command> pathCommands = getPathPlannerDriveCommandGroup(drivetrain,"BottomGrid2_5PieceTaxiConeVision", new PathConstraints[]{
                new PathConstraints(2,1),
                new PathConstraints(4,2)
        }, true);
        limelight.setPipeline(Limelight.Pipeline.GAME_PIECE);
        setName("BottomGrid2.5PieceTaxiConeVision");
        addCommands(
                Commands.deadline(
                        new SetArmPosition(arm, Constants.Auto.highCube.approach),
                        Commands.sequence(
                                new ProxyCommand(() -> new WaitCommand((arm.timeToState(Constants.Auto.highCube.approach)-150)/1000)),
                                new GrabberOuttakeCommand(intake, 0.5)
                        )
                ),
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
                                        new TurnToGamePiece(drivetrain, null, TurnToGamePiece.GamePiece.CUBE ),
                                        new DriveToGamePiece(drivetrain, () -> Constants.Auto.autoDriveDistance)        
                                ),
                                Commands.parallel(
                                        new SetArmPosition(arm, Constants.Auto.midCube.approach),
                                        pathCommands.get(1)
                                )
                        )
                ),
                new GrabberOuttakeCommand(intake,1).withTimeout(0.1),
                Commands.parallel(
                        Commands.sequence(
                                new ResetArmPositionCommand(arm),
                                new PivotCommand(arm, Constants.Auto.armDownBackwards)
                        ),
                        pathCommands.get(2)
                ),
                Commands.parallel(
                        new GrabberIntakeCommand(intake),
                        Commands.parallel(
                                new TurnToGamePiece(drivetrain, null, TurnToGamePiece.GamePiece.CONE ),
                                new DriveToGamePiece(drivetrain, () -> Constants.Auto.autoDriveDistance)
                        )
                )
//                new AutoGridScore(drivetrain, arm, Constants.Auto.midCube.withPolePosition(PolePosition.CENTER), intake, null, false)

        );
    }


}