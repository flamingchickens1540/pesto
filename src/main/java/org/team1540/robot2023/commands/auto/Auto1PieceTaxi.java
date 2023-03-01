package org.team1540.robot2023.commands.auto;

import org.team1540.robot2023.Constants;
import org.team1540.robot2023.commands.arm.Arm;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.commands.grabber.WheeledGrabber;
import org.team1540.robot2023.utils.AutoCommand;
import org.team1540.robot2023.utils.PolePosition;
import org.team1540.robot2023.utils.ScoringGridLocation;

public class Auto1PieceTaxi extends AutoCommand {
    public Auto1PieceTaxi(Drivetrain drivetrain, Arm arm, WheeledGrabber intake, ScoringGridLocation.OuterGrid grid) {
        addCommands(
                new AutoGridScore(drivetrain, arm, PolePosition.CENTER, Constants.Auto.highCube, intake),
                getPathPlannerDriveCommand(drivetrain, grid.getPathName("1PieceTaxi"))
        );
    }
}
