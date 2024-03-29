package org.team1540.robot2023.commands.auto.sequence.bottom;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
import org.team1540.robot2023.utils.AutoCommand;
import org.team1540.robot2023.utils.PolePosition;

import java.util.List;

public class AutoBottomGrid2_5PieceTaxi extends AutoCommand {
    public AutoBottomGrid2_5PieceTaxi(Drivetrain drivetrain, Arm arm, WheeledGrabber intake) {
        List<Command> pathCommands = getPathPlannerDriveCommandGroup(drivetrain,"BottomGrid2_5PieceTaxi", new PathConstraints[]{
                new PathConstraints(2,1),
                new PathConstraints(4,2)
        }, false);
        setName("BottomGrid2.5PieceTaxi");
        addCommands(
                new AutoCube(drivetrain, arm, Constants.Auto.highCube.withPolePosition(PolePosition.CENTER), intake,null,false),
                Commands.parallel(
                        new GrabberIntakeCommand(intake),
                        Commands.sequence(
                                pathCommands.get(0),
                                Commands.parallel(
                                    pathCommands.get(1),
                                    Commands.sequence(
                                            new ResetArmPositionCommand(arm),
                                            new PivotCommand(arm, Constants.Auto.armDownBackwards)
                                    )
                                ),
                                Commands.parallel(
                                        new SetArmPosition(arm, Constants.Auto.midCube.approach),
                                    pathCommands.get(2)
                                )
                        )
                ),
                new GrabberOuttakeCommand(intake,1),
                Commands.parallel(
                new ResetArmPositionCommand(arm),
                        pathCommands.get(3)
                )
//                new AutoGridScore(drivetrain, arm, Constants.Auto.midCube.withPolePosition(PolePosition.CENTER), intake, null, false)

        );
    }


}