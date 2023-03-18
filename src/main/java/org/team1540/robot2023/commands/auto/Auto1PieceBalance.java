package org.team1540.robot2023.commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import org.team1540.robot2023.Constants;
import org.team1540.robot2023.commands.arm.Arm;
import org.team1540.robot2023.commands.arm.ExtensionCommand;
import org.team1540.robot2023.commands.arm.RetractAndPivotCommand;
import org.team1540.robot2023.commands.drivetrain.AutoBalanceCommand;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.commands.grabber.GrabberOuttakeCommand;
import org.team1540.robot2023.commands.grabber.WheeledGrabber;
import org.team1540.robot2023.utils.AutoCommand;
import org.team1540.robot2023.utils.ScoringGrid;

public class Auto1PieceBalance extends AutoCommand {
    public Auto1PieceBalance(Drivetrain drivetrain, Arm arm, WheeledGrabber intake, ScoringGrid grid) {
        addCommands(
//                new AutoGridScore(drivetrain, arm, Constants.Auto.highCube.withPolePosition(PolePosition.CENTER), intake),
                new RetractAndPivotCommand(arm, Constants.Auto.reverseHighCube.approach.getRotation2d()),
                new ExtensionCommand(arm, Constants.Auto.reverseHighCube.approach),
                new GrabberOuttakeCommand(intake).withTimeout(0.3),
                new RetractAndPivotCommand(arm, Rotation2d.fromDegrees(45)),
                getPathPlannerDriveCommand(drivetrain, grid.getPathName("1PieceBalance")),
                new AutoBalanceCommand(drivetrain)
        );
    }
}
