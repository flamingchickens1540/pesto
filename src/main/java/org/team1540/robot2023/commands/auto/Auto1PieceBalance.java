package org.team1540.robot2023.commands.auto;

import org.team1540.robot2023.Constants;
import org.team1540.robot2023.commands.arm.Arm;
import org.team1540.robot2023.commands.drivetrain.AutoSideBalanceCommand;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.commands.grabber.WheeledGrabber;
import org.team1540.robot2023.utils.AutoCommand;
import org.team1540.robot2023.utils.PolePosition;
import org.team1540.robot2023.utils.ScoringGrid;

public class Auto1PieceBalance extends AutoCommand {
    public Auto1PieceBalance(Drivetrain drivetrain, Arm arm, WheeledGrabber intake, ScoringGrid grid) {
        addCommands(

                new AutoGridScore(drivetrain, arm, Constants.Auto.highCube.withPolePosition(PolePosition.CENTER), intake),
                getPathPlannerDriveCommand(drivetrain, grid.getPathName("1PieceBalance")),
                new AutoSideBalanceCommand(drivetrain)
        );
    }
}
