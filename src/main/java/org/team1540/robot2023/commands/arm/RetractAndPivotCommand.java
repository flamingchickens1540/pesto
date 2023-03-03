package org.team1540.robot2023.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2023.utils.ArmState;

public class RetractAndPivotCommand extends SequentialCommandGroup {
    public RetractAndPivotCommand(Arm arm, ArmState armState) {
        this(arm, armState.getRotation2d());
    }
    public RetractAndPivotCommand(Arm arm, Rotation2d setpoint) {
        addCommands(
                new ExtensionCommand(arm, 0),
                new RetractExtension(arm),
                new PivotCommand(arm, setpoint)
        );
        addRequirements(arm);
    }
}
