package org.team1540.robot2023.commands.auto;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;
import org.team1540.robot2023.Constants;
import org.team1540.robot2023.commands.arm.Arm;
import org.team1540.robot2023.commands.arm.RetractAndPivotCommand;
import org.team1540.robot2023.commands.drivetrain.AutoBalanceCommand;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.commands.grabber.WheeledGrabber;
import org.team1540.robot2023.utils.AutoCommand;
import org.team1540.robot2023.utils.PolePosition;

public class AutoMiddleGrid1PieceBalance extends AutoCommand {
    public AutoMiddleGrid1PieceBalance(Drivetrain drivetrain, Arm arm, WheeledGrabber intake) {
        addCommands(
//                new AutoGridScore(drivetrain, arm, Constants.Auto.highCube.withPolePosition(PolePosition.CENTER), intake),
                new AutoGridScore(drivetrain, arm, Constants.Auto.highCube.withPolePosition(PolePosition.CENTER), intake, null, false),
                new RetractAndPivotCommand(arm, Rotation2d.fromDegrees(-45)),
                getPathPlannerDriveCommand(drivetrain, "MiddleGrid1PieceBalance", new PathConstraints(1, 1), false),
                new AutoBalanceCommand(drivetrain)
        );
    }
}
