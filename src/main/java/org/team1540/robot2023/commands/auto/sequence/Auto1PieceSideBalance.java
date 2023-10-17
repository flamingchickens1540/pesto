package org.team1540.robot2023.commands.auto.sequence;

import org.team1540.robot2023.Constants;
import org.team1540.robot2023.commands.arm.Arm;
import org.team1540.robot2023.commands.auto.AutoCube;
import org.team1540.robot2023.commands.drivetrain.AutoSideBalanceCommand;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.commands.grabber.WheeledGrabber;
import org.team1540.robot2023.utils.AutoCommand;
import org.team1540.robot2023.utils.PolePosition;

public class Auto1PieceSideBalance extends AutoCommand {
    public Auto1PieceSideBalance(Drivetrain drivetrain, Arm arm, WheeledGrabber intake) {
        addCommands(
                new AutoCube(drivetrain, arm, Constants.Auto.highCube.withPolePosition(PolePosition.CENTER), intake, null, true),
                getPathPlannerDriveCommand(drivetrain, "MiddleGrid1PieceSideBalance"),
                new AutoSideBalanceCommand(drivetrain)
        );
    }
}
