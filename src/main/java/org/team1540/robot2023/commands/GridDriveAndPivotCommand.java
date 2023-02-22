package org.team1540.robot2023.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2023.commands.arm.Arm;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.commands.drivetrain.ProxiedGridDriveCommand;
import org.team1540.robot2023.utils.PolePosition;

public class GridDriveAndPivotCommand extends SequentialCommandGroup{



    public GridDriveAndPivotCommand(Drivetrain drivetrain, PolePosition position, Arm arm, Rotation2d setpoint){
        addCommands(
            new ProxiedGridDriveCommand(drivetrain, position)
//            new RetractAndPivotCommand(arm, setpoint)
        );

    }
}
