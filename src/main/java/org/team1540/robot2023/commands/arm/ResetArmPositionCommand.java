package org.team1540.robot2023.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ResetArmPositionCommand extends SequentialCommandGroup {
    public ResetArmPositionCommand(Arm arm) {
        addCommands(
                new InstantCommand(arm::stopAll),
                new RetractAndPivotCommand(arm, Rotation2d.fromDegrees(0)),
                new InstantCommand(arm::resetAngle)
        );
    }
}
