package org.team1540.robot2023.commands.auto;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import org.team1540.robot2023.Constants;
import org.team1540.robot2023.commands.arm.Arm;
import org.team1540.robot2023.commands.arm.RetractAndPivotCommand;
import org.team1540.robot2023.commands.drivetrain.AutoBalanceCommand;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.commands.grabber.WheeledGrabber;
import org.team1540.robot2023.utils.AutoCommand;
import org.team1540.robot2023.utils.PolePosition;

public class AutoBottomGrid1PieceBalance extends AutoCommand {
    public AutoBottomGrid1PieceBalance(Drivetrain drivetrain, Arm arm, WheeledGrabber intake) {
        addCommands(
//                new AutoGridScore(drivetrain, arm, Constants.Auto.highCube.withPolePosition(PolePosition.CENTER), intake),
                new AutoGridScore(drivetrain, arm, Constants.Auto.highCube.withPolePosition(PolePosition.CENTER), intake, null, false),
                Commands.parallel(
                        new RetractAndPivotCommand(arm, Rotation2d.fromDegrees(-45)),
                        getPathPlannerDriveCommand(drivetrain, "BottomGrid1PieceBalance", new PathConstraints(4, 2), false)
                ),
                new AutoBalanceCommand(drivetrain)
        );
    }
}
