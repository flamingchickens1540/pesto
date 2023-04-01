package org.team1540.robot2023.commands.auto;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.team1540.robot2023.Constants;
import org.team1540.robot2023.commands.arm.Arm;
import org.team1540.robot2023.commands.arm.RetractAndPivotCommand;
import org.team1540.robot2023.commands.drivetrain.AutoBalanceCommand;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.commands.grabber.WheeledGrabber;
import org.team1540.robot2023.utils.AutoCommand;
import org.team1540.robot2023.utils.PolePosition;

import java.util.List;

public class AutoMiddleGrid1PieceTaxiBalance extends AutoCommand {
    public AutoMiddleGrid1PieceTaxiBalance(Drivetrain drivetrain, Arm arm, WheeledGrabber intake) {
        List<Command> commands = getPathPlannerDriveCommandGroup(drivetrain, "MiddleGrid1PieceTaxiBalance", new PathConstraints(1, 1), false);
        addCommands(
//                new AutoGridScore(drivetrain, arm, Constants.Auto.highCube.withPolePosition(PolePosition.CENTER), intake),
                new AutoGridScore(drivetrain, arm, Constants.Auto.highCube.withPolePosition(PolePosition.CENTER), intake, null, false),
                new RetractAndPivotCommand(arm, Rotation2d.fromDegrees(45)),
                commands.get(0),
                Commands.parallel(
                    new RetractAndPivotCommand(arm, Rotation2d.fromDegrees(-45)),
                    commands.get(1)
                ),
                new AutoBalanceCommand(drivetrain)
        );
    }
}
