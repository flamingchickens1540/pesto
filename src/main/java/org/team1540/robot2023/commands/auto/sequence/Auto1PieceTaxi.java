package org.team1540.robot2023.commands.auto.sequence;

import org.team1540.robot2023.Constants;
import org.team1540.robot2023.commands.arm.Arm;
import org.team1540.robot2023.commands.auto.AutoCube;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.commands.grabber.WheeledGrabber;
import org.team1540.robot2023.utils.AutoCommand;
import org.team1540.robot2023.utils.PolePosition;
import org.team1540.robot2023.utils.ScoringGridLocation;

public class Auto1PieceTaxi extends AutoCommand {
    public Auto1PieceTaxi(Drivetrain drivetrain, Arm arm, WheeledGrabber intake, ScoringGridLocation.OuterGrid grid) {
        addCommands(
                new AutoCube(drivetrain, arm, Constants.Auto.highCube.withPolePosition(PolePosition.CENTER), intake, true),
                getPathPlannerDriveCommand(drivetrain, grid.getPathName("1PieceTaxi"))
        );
    }
}
