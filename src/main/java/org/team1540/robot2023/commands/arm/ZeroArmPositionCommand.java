package org.team1540.robot2023.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ZeroArmPositionCommand extends SequentialCommandGroup {
    public ZeroArmPositionCommand(Arm arm) {
        addCommands(
//                new InstantCommand(arm::stopAll),
//                new RetractAndPivotCommand(arm, Rotation2d.fromDegrees(-20)).withTimeout(1),
                new InstantCommand(arm::resetToGyro)
        );
    }
}
