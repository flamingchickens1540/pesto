package org.team1540.robot2023.commands.auto;

import org.team1540.robot2023.Constants;
import org.team1540.robot2023.commands.arm.Arm;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.utils.AutoCommand;
import org.team1540.robot2023.utils.PolePosition;

public class BottomGrid1PieceTaxi extends AutoCommand {
    public BottomGrid1PieceTaxi(Drivetrain drivetrain, Arm arm) {
        addCommands(
                new GridDriveAndPivotCommand(drivetrain, PolePosition.CENTER, arm, Constants.Auto.armHighCube),
                getPathPlannerDriveCommand(drivetrain, "BottomGrid1PieceTaxi")
        );
    }
}
